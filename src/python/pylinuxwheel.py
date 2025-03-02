#!/usr/bin/python3
from urllib.error import HTTPError, URLError
import sys
import gi
import os
import logging
import subprocess
import json
import time
import gettext
import locale
import getpass
import urllib.request
from evdev import ecodes, InputDevice, ff
import pyudev
from re import match

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk
from gi.repository import GLib

# create logger
logger = logging.getLogger('pyLinuxWheel')
logging.basicConfig(level=logging.DEBUG)
APP = 'pyLinuxWheel'


class UtilPath:

    default_preferences = "pyLinuxWheel_preferences"
    default_profile = "default"
    version = "0.6.1"
    branch="master"

    def __init__(self):
        self.directory_name="pyLinuxWheel"
        self.rules_file = '99-logitech-wheel-perms.rules'

    def config_path_exists(self):
        path = self.get_config_path()
        if path is not None:
            return os.path.exists(self.get_config_path())
        else:
            return False

    def get_config_path(self):
        if os.path.exists(os.path.join(GLib.get_user_config_dir(), self.directory_name)):
            return os.path.join(GLib.get_user_config_dir(),  self.directory_name)
        else:
            logger.warning("there is not a configuration directory")
            return None

    def create_config_path(self):
        if not self.config_path_exists():
            os.makedirs(os.path.join(GLib.get_user_config_dir(), self.directory_name))
        if not os.path.exists(self.get_profiles_path()):
            os.makedirs(self.get_profiles_path())

    def get_data_path(self):
        if getattr(sys, 'frozen', False):
            base_folder = sys._MEIPASS
            logger.info("base_folder %s" % base_folder)
            return base_folder + "/usr/share/" + self.directory_name + "/data"
        elif os.path.exists("/usr/share/" + self.directory_name + "/data"):
            return "/usr/share/" + self.directory_name + "/data"
        elif os.path.exists("data"):
            return os.path.abspath("data")
        else:
            base_folder = os.path.dirname(__file__)
            logger.info("base_folder %s" % base_folder)
            if os.path.exists(base_folder + "/usr/share/" + self.directory_name + "/data"):
                return base_folder + "/usr/share/" + self.directory_name + "/data"
            else:
                return base_folder + "/data"

    def get_po_path(self):
        if getattr(sys, 'frozen', False):
            base_folder = sys._MEIPASS
            logger.info("base_folder %s" % base_folder)
            return base_folder + "/usr/share/locale/"
        elif os.path.exists("locale"):
            return os.path.abspath("locale/")
        else:
            base_folder =  os.path.dirname(__file__)
            logger.info("base_folder %s" % base_folder)
            if os.path.exists(base_folder + "/usr/share/locale/"):
                return base_folder + "/usr/share/locale/"
            elif os.path.exists(base_folder + "/locale/"):
                return base_folder + "/locale/"
            else:
                return "/usr/share/locale/"

    def get_glade_files_path(self):
        return os.path.join(self.get_data_path(), "glade_files")

    def get_profiles_path(self):
        return os.path.join(self.get_config_path(), "profiles")

    def get_wheels_path(self):
        return os.path.join(self.get_data_path(), "wheels")

    def get_rules_path(self):
        return os.path.join(self.get_data_path(), "rules")

    def check_logitech_rules(self):
        return os.path.exists(os.path.join('/etc', 'udev', 'rules.d',  self.rules_file))

    def download_file(self,url, file_name):
        downloaded= False
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': 'Mozilla'})) as response, open(file_name, 'wb') as out_file:
                data = response.read()  # a `bytes` object
                out_file.write(data)
        except HTTPError as e:
            # do something
            logger.error(e)
        except URLError as e:
            logger.error(e)
        except IOError as e:
            logger.error(e)
        else:
            logger.info("udev rules has been downloaded")
            downloaded = True
        return downloaded

    def download_udev_rules(self):
        url = "https://gitlab.com/OdinTdh/pyLinuxWheel/raw/" + self.branch + "/data/rules/" + self.rules_file
        return self.download_file(url, os.path.join(util_path.get_config_path(), self.rules_file))

    def copy_logitech_rules(self):
        if self.download_udev_rules():
            command = "cp -f "+ os.path.join(util_path.get_config_path(), self.rules_file) + " " + os.path.join('/', 'etc', 'udev','rules.d/') + self.rules_file + " && udevadm control --reload-rules && udevadm trigger"
            logger.info(command)
            result = subprocess.run(["pkexec", "/bin/bash", "-c", command])
            logger.info("copy_logitech_rules return " + str(result.returncode))
            return result.returncode
        else:
            return 1


class PreferencesService:

    def write_preferences(self,preferences_data):
        util_path = UtilPath()
        with open(os.path.join(util_path.get_config_path(), preferences_data["file_name"] + '.json'), 'w') as outfile:
            json.dump(preferences_data, outfile)

    def read_preferences(self, name):
        util_path = UtilPath()
        preferences_path=os.path.join(util_path.get_config_path(),name + '.json')
        with open(preferences_path) as json_data:
            data = json.load(json_data)
            logger.debug(data)
            return data

    def check_preferences(self,name):
        util_path = UtilPath()
        return os.path.exists(os.path.join(util_path.get_config_path(),name + '.json'))

    def write_profile(self, profile):
        util_path = UtilPath()
        with open(os.path.join(util_path.get_profiles_path(), profile["file_name"] + '.json'), 'w') as outfile:
            json.dump(profile, outfile)

    def export_profile(self,source,target_path,format="json"):
        util_path = UtilPath()
        try:
            data=self.read_profile(source)
            with open(target_path+'.'+format, 'w') as outfile:
                json.dump(data, outfile)
        except FileNotFoundError as fileError:
            logger.error(fileError)

    def import_profile(self, source_path):
        util_path = UtilPath()
        data = None
        profile_name=None
        with open(source_path) as json_data:
            data = json.load(json_data)
            logger.debug(data)
        if data is not None:
            profile_name=data.get("file_name")
            if profile_name != "default":
                self.write_profile(data)
            else:
                logger.error("You can not override the default profile.")
        return profile_name

    def read_profile(self, name):
        util_path = UtilPath()
        profiles_path = os.path.join(util_path.get_profiles_path(), name + '.json')
        with open(profiles_path) as json_data:
            data = json.load(json_data)
            logger.debug(data)
            return data

    def delete_profile(self, profile):
        util_path = UtilPath()
        logger.info("delete_profile "+profile)
        os.remove(os.path.join(util_path.get_profiles_path(), profile + '.json'))

    def list_profiles(self):
        logger.info(os.listdir(path=util_path.get_profiles_path()))
        return os.listdir(path=util_path.get_profiles_path())

    def check_profile(self, name):
        util_path = UtilPath()
        return os.path.exists(os.path.join(util_path.get_profiles_path(), name + '.json'))


class EvdevWheelService:

    logitech_model_dict = {'C202': {"description": "Logitech WingMan Formula (Yellow) (USB)", "max_degrees": 180,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': False,
                                                           'alternate_modes': False, 'ff_gain': True, 'ff_autocenter':True}},
                           'C20E': {"description": "Logitech WingMan Formula GP", "max_degrees": 180,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': False,
                                                           'alternate_modes': False, 'ff_gain': False, 'ff_autocenter':False}},
                           'C293': {"description": "Logitech WingMan Formula Force GP USB", "max_degrees": 180,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': False,
                                                           'alternate_modes': False, 'ff_gain': True, 'ff_autocenter':True}},
                           'CA04': {"description": "Logitech Racing Wheel USB", "max_degrees": 180,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': False,
                                                           'alternate_modes': False, 'ff_gain': True, 'ff_autocenter':True}},
                           'CA03': {"description": "Logitech MOMO Racing USB", "max_degrees": 270,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 1,
                                    "brake": 2,
                                    "clutch": None,
                                    "supported_features": {'range': True, 'combine_pedals': False,
                                                           'alternate_modes': False, 'ff_gain': True, 'ff_autocenter':True}},
                           'C295': {"description": "Logitech MOMO Force USB", "max_degrees": 270,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 1,
                                    "brake": 2,
                                    "clutch": None,
                                    "supported_features": {'range': True, 'combine_pedals': False,
                                                           'alternate_modes': False, 'ff_gain': True, 'ff_autocenter':True}},
                           'C294': {"description": "Logitech Driving Force USB", "max_degrees": 270,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': True,
                                                           'alternate_modes': False, 'ff_gain': True, 'ff_autocenter':True}},
                           'C29A': {"description": "Logitech Driving Force GT USB", "max_degrees": 900,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 1,
                                    "brake": 2,
                                    "clutch": None,
                                    "supported_features": {'range': True, 'combine_pedals': True,
                                                           'alternate_modes': True, 'ff_gain': True, 'ff_autocenter':True}},
                           'C298': {"description": "Logitech Driving Force Pro USB", "max_degrees": 900,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 1,
                                    "brake": 5,
                                    "clutch": None,
                                    "supported_features": {'range': True, 'combine_pedals': True,
                                                           'alternate_modes': True, 'ff_gain': True, 'ff_autocenter':True}},
                           'C299': {"description": "Logitech G25 Racing Wheel USB", "max_degrees": 900,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': True,
                                                           'alternate_modes': True, 'ff_gain': True, 'ff_autocenter':True}},
                           'C29B': {"description": "Logitech G27 Racing Wheel USB", "max_degrees": 900,
                                    "min_degrees": 40,
                                    "precision": 16384,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': True,
                                                           'alternate_modes': True, 'ff_gain': True, 'ff_autocenter':True}},
                           'C24F': {"description": "Logitech G29 Racing Wheel USB", "max_degrees": 900,
                                    "min_degrees": 40,
                                    "precision": 65535,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch":1,
                                    "supported_features": {'range': True, 'combine_pedals': True,
                                                           'alternate_modes': True, 'ff_gain': True, 'ff_autocenter':True}},
                           'C262': {"description": "Logitech G920 Racing Wheel USB", "max_degrees": 900,
                                    "min_degrees": 40,
                                    "precision": 65535,
                                    "wheel": 0,
                                    "throttle": 2,
                                    "brake": 5,
                                    "clutch": 1,
                                    "supported_features": {'range': True, 'combine_pedals': False,
                                                           'alternate_modes': False, 'ff_gain': True, 'ff_autocenter':True}}
                           }

    def __init__(self):
        self._wheel_device = None
        self._driver_path = None
        self._gain = None
        self._autocenter = None

    def get_device(self):
        return self._wheel_device

    def get_wheel_model(self):
        if self.supported_wheel():
            info = self.get_device().info
            value = str(hex(info.product)).replace('0x', '', 1)
            return value.upper()
        else:
            return None

    def supported_features(self):
        if self.logitech_model_dict.get(self.get_wheel_model()) is not None:
            return self.logitech_model_dict.get(self.get_wheel_model()).get("supported_features")
        else:
            return {'range': False, 'combine_pedals': False, 'alternate_modes': False, 'ff_gain': False, 'ff_autocenter':False}

    @property
    def max_degrees(self):
        max_range = 180
        if self.logitech_model_dict.get(self.get_wheel_model()) is not None:
            max_range = self.logitech_model_dict.get(self.get_wheel_model()).get("max_degrees")
        return max_range

    @property
    def min_degrees(self):
        min_range = 40
        if self.logitech_model_dict.get(self.get_wheel_model()) is not None:
            min_range = self.logitech_model_dict.get(self.get_wheel_model()).get("min_degrees")
        return min_range

    @property
    def driver_path(self):
        return self._driver_path

    def supported_wheel(self, force_validation=False):
        if force_validation:
            self.sync_wheel()
        if self.get_device() is not None:
            return True
        else:
            return False

    def sync_wheel(self):
        self._wheel_device = None
        self._driver_path = None
        try:
            context = pyudev.Context()
            for device in context.list_devices(subsystem='input', ID_INPUT_JOYSTICK=1):
                if device.get('ID_VENDOR_ID') == "046d" and device.get('ID_MODEL_ID') is not None and device.get(
                        'ID_MODEL_ID').upper() in self.logitech_model_dict.keys() and "event" in device.get('DEVPATH'):
                    self._driver_path = str(device.sys_path).split('event')[0] + "device/"
                    self._wheel_device = InputDevice(device.get('DEVNAME'))
                    break
        except:
            logger.warning("EvdevWheelService does not find a Logitech wheel connected")

    @property
    def model_wheels_supported(self):
        return ['DFP', 'DFGT', 'G25', 'G27', 'G29', 'G920', 'MR','MF', 'WINGMAN', 'DF']

    def is_on_emulation_mode(self):
        if self.supported_wheel() and self.supported_features()["alternate_modes"]:
            if self.get_alternate_modes() is not None:
                list_modes = self.get_alternate_modes()
                find = [x for x in list_modes if "*" in x and "native" in x]
                if find is not None and len(find) > 0:
                    return False
                else:
                    return True
            else:
                return False
        else:
            return False

    def name_wheel(self):
        if self.supported_wheel():
            if self.supported_features()['alternate_modes']:
                return self._read_from_driver('real_id')
            else:
                return self.get_device().name
        else:
            return None

    def current_degrees(self):
        degrees = None
        if self.supported_features()["range"]:
            try:
                degrees = int(self._read_from_driver('range'))
            except ValueError as verror:
                logger.error(verror)
            except TypeError as terror:
                logger.error(terror)
        else:
            logger.info("range is not supported")
        return degrees

    def write_driver_degrees(self, param_degrees):
        if self.supported_features()["range"]:
            degrees = int(param_degrees)
            if not self.min_degrees - 1 < degrees < self.max_degrees + 1:
                raise ValueError(
                    'Error, Valid range is between ' + str(self.min_degrees) + ' and ' + str(self.max_degrees))
            else:
                self._write_to_driver('range', param_degrees)
        else:
            logger.info("range is not supported")

    def write_driver_combine_pedals(self, value):
        logger.info("write_driver_combine_pedals")
        if self.supported_features()["combine_pedals"]:
            if value:
                self._write_to_driver('combine_pedals', 1)
            else:
                self._write_to_driver('combine_pedals', 0)
        else:
            logger.info("combine_pedals is not supported")

    def get_combine_pedals(self):
        value = False
        if self.supported_features()["combine_pedals"]:
            try:
                combine_pedals = int(self._read_from_driver('combine_pedals'))
                if combine_pedals != 0:
                    value = True
            except ValueError as verror:
                logger.error(verror)
            except TypeError as terror:
                logger.error(terror)
        else:
            logger.info("combine_pedals is not supported")
        return value

    def get_alternate_modes(self):
        list_modes = None
        if self.supported_features()["alternate_modes"]:
            parameter = "alternate_modes"
            print(self.driver_path)
            try:
                with open(os.path.join(self.driver_path, parameter), 'r') as alternate_modes_file:
                    list_modes = [line.rstrip('\n') for line in alternate_modes_file]
                #logger.info(list_modes)
            except Exception as e:
                logger.error('Error reading drivers properties ' + str(e))
        else:
            logger.info("alternate_modes is not supported")
        return list_modes

    def write_alternate_modes(self, mode):
        logger.info("write_alternate_modes")
        if self.supported_features()["alternate_modes"]:
            list_modes = self.get_alternate_modes()
            if list_modes is not None:
                find = [x for x in list_modes if mode in x and "*" in x]
                if find is not None and len(find) > 0:
                    logger.info("Is the same mode, no need to override")
                else:
                    logger.info("Different alternate mode, override")
                    self._write_to_driver('alternate_modes', mode)
                    time.sleep(8)
                    # a change in alternate modes update the sys_path so we have to sync again
                    self.sync_wheel()
        else:
            logger.info("alternate_modes is not supported")

    def write_autocenter(self, autocenter_force=0):
        logger.info("write_autocenter %i" % autocenter_force)
        logger.info (self.get_device().capabilities(verbose=True))
        if self.supported_wheel() and self.supported_features()["ff_autocenter"]  and ecodes.EV_FF in self.get_device().capabilities():
            input_device = self.get_device()
            if autocenter_force is None:
                autocenter_force = 0
            elif autocenter_force > 100:
                autocenter_force = 100
            elif autocenter_force < 0:
                autocenter_force = 0
            self._autocenter = autocenter_force
            value = 0xFFFF * autocenter_force / 100
            self.get_device().write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, int(value))

    def write_gain(self, gain=100):
        logger.info("write_gain %i" % gain)
        if self.supported_wheel() and self.supported_features()["ff_gain"] and ecodes.EV_FF in self.get_device().capabilities():
            if gain is None:
                gain = 100
            elif gain > 100:
                gain = 100
            elif gain < 0:
                gain = 0
            self._gain = gain
            value = 0xFFFF * gain / 100
            self.get_device().write(ecodes.EV_FF, ecodes.FF_GAIN, int(value))

    def listen_events(self, ui):
        ui.callback(self.get_device().read_one())

    def test_rumble(self):
        if self.supported_wheel() and ecodes.EV_FF in self.get_device().capabilities():
            input_device = self.get_device()
            if ecodes.EV_FF in input_device.capabilities():
                envelope = ff.Envelope(500, 32767, 500, 32767)
                constant = ff.Constant(32767, envelope)
                duration_ms = 1000
                effect = ff.Effect(
                    ecodes.FF_CONSTANT, -1, 0xC000,
                    ff.Trigger(0, 0),
                    ff.Replay(duration_ms, 1),
                    ff.EffectType(ff_constant_effect=constant)
                )
                effect_id = input_device.upload_effect(effect)
                repeat_count = 4
                input_device.write(ecodes.EV_FF, effect_id, repeat_count)
                time.sleep(4)
                input_device.erase_effect(effect_id)

    def get_gain(self):
        if self.supported_features()["ff_gain"]:
            if self._gain is not None:
                return self._gain
            preferences_service = PreferencesService()
            preference=preferences_service.read_preferences(UtilPath.default_preferences)
            if preference["load_last_value"] and preferences_service.check_profile(UtilPath.default_profile):
                profile = preferences_service.read_profile(UtilPath.default_profile)
                if profile.get('ff_gain') is not None:
                    return profile['ff_gain']
                else:
                    return 100
            else:
                return 100
        else:
            logger.info("get_gain is not supported")
            return 100

    def get_autocenter(self):
        if self.supported_features()["ff_autocenter"]:
            if self._autocenter is not None:
                return self._autocenter
            preferences_service = PreferencesService()
            preference = preferences_service.read_preferences(UtilPath.default_preferences)
            if preference["load_last_value"] and preferences_service.check_profile(UtilPath.default_profile):
                profile = preferences_service.read_profile(UtilPath.default_profile)
                if profile.get('ff_autocenter') is not None:
                    return profile['ff_autocenter']
                else:
                    return 0
            else:
                return 0
        else:
            logger.info("get_autocenter is not supported")
            return 0

    def synchronize_from_driver_to_profile(self, profile_name="default"):
        logger.info("synchronize_from_driver_to_profile")
        if self.supported_wheel():
            preferences_service = PreferencesService()
            combine = None
            if self.get_combine_pedals():
                combine = 1
            else:
                combine = 0
            list_modes = self.get_alternate_modes()
            selected_mode = None
            if list_modes is not None:
                for mode in list_modes:
                    if "*" in mode and "native" not in mode:
                        selected_mode = mode.rstrip("\n")
                        selected_mode=selected_mode.replace("*"," ")
                        selected_mode = selected_mode.split(":")
                        selected_mode = selected_mode[0]
                        logger.info(selected_mode)
            gain = self.get_gain()
            autocenter = self.get_autocenter()
            profile = {"file_name": profile_name, "version": UtilPath.version, "range": self.current_degrees(), "combine_pedals": combine, "alternate_modes": selected_mode, 'ff_gain':gain, 'ff_autocenter':autocenter}
            preferences_service.write_profile(profile)

    def synchronize_from_profile_to_driver(self, profile_name="default"):
        logger.info("synchronize_from_profile_to_driver")
        preferences_service = PreferencesService()
        if self.supported_wheel() and preferences_service.check_profile(profile_name):
            profile = preferences_service.read_profile(profile_name)
            if profile.get("alternate_modes") is not None:
                self.write_alternate_modes(profile["alternate_modes"])
            else:
                self.write_alternate_modes("native")
            if profile.get("range") is not None:
                if profile["range"] > self.max_degrees:
                    self.write_driver_degrees(self.max_degrees)
                elif profile["range"] < self.min_degrees:
                    self.write_driver_degrees(self.min_degrees)
                else:
                    self.write_driver_degrees(profile["range"])
            if profile.get("combine_pedals") is not None:
                self.write_driver_combine_pedals(profile["combine_pedals"])
            if profile.get('ff_gain') is not None:
                self.write_gain(profile['ff_gain'])
            if profile.get('ff_autocenter') is not None:
                self.write_autocenter(profile['ff_autocenter'])

    def _write_to_driver(self, parameter, value):
        try:
            with open(os.path.join(self.driver_path, parameter), 'w') as property:
                property.write(str(value))
        except IOError as error:
            logger.error(error)
            raise ValueError('Error trying to write in the driver')

    def _read_from_driver(self, parameter):
        property_value = None
        try:
            with open(os.path.join(self.driver_path, parameter), 'r') as property:
                property_value = str(property.readline())
        except Exception:
            logger.info('Error reading drivers properties ')
        return property_value



class MockWheelService:

    @property
    def max_degrees(self):
        return 180


    @property
    def min_degrees(self):
        return 0

    @property
    def model_wheels_supported(self):
        return ['Unsupported']


    def name_wheel(self):
        return "Unsupported model"


    def get_wheel_model(self):
        logger.info("Unsupported model do not have model id")
        return None


    def supported_wheel(self,force_validation=False):
        logger.info("Unsupported model")
        return False


    def get_autocenter(self):
        logger.info("Unsupported model does not support get_autocenter")
        return 0


    def get_gain(self):
        logger.info("Unsupported model does not support get_gain")
        return 100


    def get_alternate_modes(self):
        logger.info("Unsupported model does not support get_alternate_modes")
        return None

    def is_on_emulation_mode(self):
        logger.info("Unsupported model does not support emulation mode")
        return False

    def get_combine_pedals(self):
        logger.info("Unsupported model does not support get_combine_pedals")
        return False

    def current_degrees(self):
        logger.info("Unsupported model does not support current_degrees")
        return 0

    def write_alternate_modes(self, mode):
        logger.info("Unsupported model does not support write_alternate_modes")

    def write_autocenter(self, autocenter_force=0):
        logger.info("Unsupported model does not support write_autocenter")

    def write_gain(self, gain=100):
        logger.info("Unsupported model does not support write_gain")

    def listen_events(self, ui):
        logger.info("Unsupported model does not support listen_events")

    def test_rumble(self):
        logger.info("Unsupported model does not support test_rumble")

    def synchronize_from_profile_to_driver(self, profile_name):
        logger.info("Unsupported model does not support synchronize_from_profile_to_driver")

    def synchronize_from_driver_to_profile(self, profile_name):
        logger.info("Unsupported model does not support synchronize_from_driver_to_profile")

    def supported_features(self):
        return {'range': False, 'combine_pedals': False, 'alternate_modes': False, 'ff_gain': False, 'ff_autocenter':False}


    def write_autocenter(self, autocenter_force=0):
        logger.info("Unsupported model does not support write_autocenter")

    def write_gain(self, gain=100):
        logger.info("Unsupported model does not support write_gain")

class DialogDummyHandler:

    def accept(self):
        logger.info("accept")
        return True

    def cancel(self):
        logger.info("cancel")
        return False


class DialogInstallRulesHandler:

    def accept(self):
        logger.info("Installing logitech wheel rules")
        response = util_path.copy_logitech_rules()
        if response == 0 :
            return True
        else:
            return False

    def cancel(self):
        logger.info("Logitech rules are not going to be installed")
        return False


class DialogUI(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="pyLinuxWheel Configurator")

    def warning(self, message, secondary_message, buttons_type, dialog_ui_handler):
        response = None
        dialog = Gtk.MessageDialog(self, 0, Gtk.MessageType.WARNING,
                                   buttons_type, message)
        dialog.format_secondary_text(secondary_message)
        widget = dialog.run()
        if widget == Gtk.ResponseType.OK:
            response = dialog_ui_handler.accept()

        elif widget == Gtk.ResponseType.CANCEL:
            response = dialog_ui_handler.cancel()

        dialog.destroy()
        return response

    def info(self, message, secondary_message, buttons_type, dialog_ui_handler):
        response = None
        dialog = Gtk.MessageDialog(self, 0, Gtk.MessageType.INFO,
                                   buttons_type, message)
        dialog.format_secondary_text(secondary_message)
        widget = dialog.run()
        if widget == Gtk.ResponseType.OK:
            response = dialog_ui_handler.accept()
        elif widget == Gtk.ResponseType.CANCEL:
            response = dialog_ui_handler.cancel()
        dialog.destroy()
        return response

class ScanService():

    instance = None


    def __new__(cls):

        if cls.instance is None:
            cls.instance = super(ScanService, cls).__new__(cls)
            cls.evdev_wheel = EvdevWheelService()
            cls.evdev_wheel.sync_wheel()
        return cls.instance

    def get_wheel(self):
        if self.evdev_wheel.supported_wheel(force_validation=True):
            wheel = self.evdev_wheel
        else:
            wheel = MockWheelService()
        logger.info("Scan Service Detect %s" % wheel.name_wheel())
        return wheel



class App:

    def __init__(self):
        self.util_path = util_path
        self.main_program = None
        self.preferences_service = PreferencesService()
        self.util_path = UtilPath()


    def pre(self):
        self.util_path.create_config_path()
        logger.info(self.util_path.get_config_path())
        file_handler = logging.FileHandler(os.path.join(self.util_path.get_config_path(), 'pyLinuxWheel.log'))
        file_handler.setLevel(level=logging.DEBUG)
        logger.addHandler(file_handler)
        logger.info(os.getcwd())
        logger.info("starting program")
        preference_data = {"file_name": UtilPath.default_preferences, "version": UtilPath.version,
                           "check_udev_rules": True, "load_last_value": True}
        if not self.preferences_service.check_preferences(UtilPath.default_preferences):
            logger.info(preference_data)
            self.preferences_service.write_preferences(preference_data)
            logger.info("preferences file created")
        configuration = self.preferences_service.read_preferences(UtilPath.default_preferences)
        new_configuration_version = False
        if configuration["version"] != UtilPath.version:
            new_configuration_version = True
            configuration["version"] = UtilPath.version
            for key in preference_data.keys():
                if key not in configuration :
                    configuration[key] = preference_data [key]
            self.preferences_service.write_preferences(configuration)
        if os.getuid() != 0 and configuration["check_udev_rules"] is True:
            dialog = DialogUI()
            if self.util_path.check_logitech_rules() :
                logger.info("Detected 99-logitech-wheel-perms.rules")
                if new_configuration_version:
                    pre_configuration_title = _("pyLinuxWheel Pre-Configuration")
                    result = dialog.info(pre_configuration_title, _(
                        " You are executing a new version of pyLinuxWheel. Do you want to update Logitech wheel udev rules?"),
                                         Gtk.ButtonsType.OK_CANCEL, DialogInstallRulesHandler())
                    post_configuration_title = _("pyLinuxWheel Post-Configuration")
                    if result:
                        dialog = DialogUI()
                        dialog.info(post_configuration_title, _("Logitech wheel udev rules has been updated"),
                                    Gtk.ButtonsType.OK, DialogDummyHandler())
                    else:
                        dialog = DialogUI()
                        dialog.warning(post_configuration_title, _(
                            "Logitech wheel udev rules are not updated"),
                                       Gtk.ButtonsType.OK, DialogDummyHandler())
            else:
                pre_configuration_title = _("pyLinuxWheel Pre-Configuration")
                result = dialog.info(pre_configuration_title, _("You are executing pyLinuxWheel without root privileges. Do you want to install Logitech wheel udev rules to execute  pyLinuxWheel as normal user ?"),
                            Gtk.ButtonsType.OK_CANCEL, DialogInstallRulesHandler())
                post_configuration_title = _("pyLinuxWheel Post-Configuration")
                if result:
                    dialog = DialogUI()
                    dialog.info(post_configuration_title, _("Logitech wheel udev rules has been installed"),
                                Gtk.ButtonsType.OK, DialogDummyHandler())
                else:
                    dialog = DialogUI()
                    dialog.warning(post_configuration_title, _("Logitech wheel udev rules are not installed. You can not change your wheel driver without root privileges and with Logitech wheel udev rules non installed"),
                                Gtk.ButtonsType.OK, DialogDummyHandler())

    def workaround_dfgt(self):
        # DFGT: Driving Force GT
        wheel_service = ScanService().get_wheel()
        if wheel_service.name_wheel() is not None and "DFGT: Driving Force GT" in wheel_service.name_wheel():
            logger.info("workaround inicialization in DFGT: Driving Force GT driver, force 270 degrees")
            wheel_service.write_driver_degrees(270)


    def run_main_ui(self):
        builder = Gtk.Builder()
        builder.set_translation_domain(APP)
        wheel = ScanService().get_wheel()
        self.main_program = MainUI(builder, wheel , self.util_path, self.preferences_service)
        self.main_program.load_window()

    def execute(self):
        self.pre()
        self.workaround_dfgt()
        self.run_main_ui()


class PreferencesUIHandler:

    def __init__(self, builder, preferences_service):
        self.builder = builder
        self.override_button = self.builder.get_object("ui_override_button")
        self.check_udev_rules_switch = self.builder.get_object("ui_check_udev_rules_start_switch")
        self.remember_last_value_saved_switch = self.builder.get_object("ui_remember_last_value_saved_switch")
        self.preferences_service = preferences_service
        self.connect_signals()

    def load_window(self):
        self.refresh()

    def refresh(self):
        logger.info("refresh ")
        preference_data = self.preferences_service.read_preferences(UtilPath.default_preferences)
        self.check_udev_rules_switch.set_active(preference_data["check_udev_rules"])
        self.remember_last_value_saved_switch.set_active(preference_data["load_last_value"])

    def connect_signals(self):
        self.override_button.connect("clicked", self.on_override_clicked)
        self.check_udev_rules_switch.connect("notify::active", self.on_ui_check_udev_rules_start_switch_activate)
        self.remember_last_value_saved_switch.connect("notify::active", self.on_ui_remember_last_value_saved_switch_activate)

    def on_override_clicked(self,button):
        logger.info("on_override_clicked")
        dialog = DialogUI()
        udev_configuration_title = _("pyLinuxWheel Udev Configuration")
        if util_path.check_logitech_rules():
            logger.info("Detected 99-logitech-wheel-perms.rules")
        result = dialog.info(udev_configuration_title,
                             _("It is necessary an Internet connection to update logitech wheel udev rules. Do you want to update logitech wheel udev rules to execute pyLinuxWheel as normal user ?"),
                             Gtk.ButtonsType.OK_CANCEL, DialogInstallRulesHandler())
        if result:
            dialog = DialogUI()
            dialog.info(udev_configuration_title, _("Logitech wheel udev rules has been updated"),
                        Gtk.ButtonsType.OK, DialogDummyHandler())
        else:
            dialog = DialogUI()
            dialog.warning(udev_configuration_title,
                           _("Logitech wheel udev rules are not update. You can not change your wheel driver without root privileges and with logitech wheel udev rules non installed"),
                           Gtk.ButtonsType.OK, DialogDummyHandler())

    def on_ui_check_udev_rules_start_switch_activate(self, button, active):
        logger.info("on_ui_check_udev_rules_start_switch_activate")
        preference_data = self.preferences_service.read_preferences(UtilPath.default_preferences)
        if button.get_active():
            preference_data["check_udev_rules"] = True
        else:
            preference_data["check_udev_rules"] = False
        self.preferences_service.write_preferences(preference_data)

    def on_ui_remember_last_value_saved_switch_activate(self, button, active):
        logger.info("on_ui_remember_last_value_saved_switch_activate")
        preference_data = self.preferences_service.read_preferences(UtilPath.default_preferences)
        if button.get_active():
            preference_data["load_last_value"] = True
        else:
            preference_data["load_last_value"] = False
        self.preferences_service.write_preferences(preference_data)

class TestUIHandler:

    def __init__(self, builder, wheel_service, preferences_service):
        self.builder = builder
        self.wheel_service = wheel_service
        self.preferences_service = preferences_service
        self.ui_scale_test_wheel = self.builder.get_object("ui_scale_test_wheel")
        self.ui_scale_test_throttle = self.builder.get_object("ui_scale_test_throttle")
        self.ui_scale_test_brake = self.builder.get_object("ui_scale_test_brake")
        self.ui_scale_test_clutch = self.builder.get_object("ui_scale_test_clutch")
        self.test_rumble_button = self.builder.get_object("ui_gtkbutton_test_rumble_ok")
        self.button_code_value = self.builder.get_object("ui_gtk_button_test_value")
        self.connect_signals()

    def connect_signals(self):
        self.test_rumble_button.connect("clicked", self.on_test_rumble_clicked)

    def load_window(self):
        self.reload()

    def get_listen_input(self):
        self.wheel_service.listen_events(self)
        return True

    def callback(self, param):
        if self.wheel_service is not None and param is not None and param.value > 0:
            if param.code != 4:
                self.button_code_value.set_text(str(param.code))
            if param.code == self.wheel_service.logitech_model_dict.get(self.wheel_service.get_wheel_model()).get("throttle", 2):
                value = float(param.value)
                value = value * 100 / 255
                self.ui_scale_test_throttle.set_value(100 - value)
            elif param.code == self.wheel_service.logitech_model_dict.get(self.wheel_service.get_wheel_model()).get("brake", 5):
                value = float(param.value)
                value = value * 100 / 255
                self.ui_scale_test_brake.set_value(100 - value)
            elif param.code == self.wheel_service.logitech_model_dict.get(self.wheel_service.get_wheel_model()).get("clutch", 1):
                value = float(param.value)
                value = value * 100 / 255
                self.ui_scale_test_clutch.set_value(100 - value)
            elif param.code == self.wheel_service.logitech_model_dict.get(self.wheel_service.get_wheel_model()).get("wheel", 0):
                value = int(param.value)
                div = 16384
                model = self.wheel_service.logitech_model_dict.get(self.wheel_service.get_wheel_model(), None)
                if model is not None:
                    if self.wheel_service.get_wheel_model() == 'C24F' and not self.wheel_service.is_on_emulation_mode():
                        div = 65535
                    else :
                        div = model.get('precision', 16384)
                value = value * self.wheel_service.current_degrees() / div
                self.ui_scale_test_wheel.set_value(value)

    def reload(self):
        self.wheel_service = ScanService().get_wheel()
        self.button_code_value.set_text("")
        self.ui_scale_test_wheel.set_range(0, self.wheel_service.max_degrees)
        self.ui_scale_test_wheel.set_value(0)
        self.ui_scale_test_wheel.clear_marks()
        self.add_marks(self.ui_scale_test_wheel,0, [0,self.wheel_service.max_degrees])
        self.ui_scale_test_throttle.set_range(0, 100)
        self.ui_scale_test_throttle.set_value(0)
        self.ui_scale_test_throttle.clear_marks()
        self.add_marks(self.ui_scale_test_throttle,0, [0, 100])
        self.ui_scale_test_brake.set_range(0, 100)
        self.ui_scale_test_brake.set_value(0)
        self.ui_scale_test_brake.clear_marks()
        self.add_marks(self.ui_scale_test_brake,0, [0, 100])
        self.ui_scale_test_clutch.set_range(0, 100)
        self.ui_scale_test_clutch.set_value(0)
        self.ui_scale_test_clutch.clear_marks()
        self.add_marks(self.ui_scale_test_clutch, 0, [0, 100])
        self.unlock_buttons()
        if self.wheel_service.supported_wheel():
            GLib.timeout_add(priority=GLib.PRIORITY_DEFAULT, interval=1,
                         function=self.get_listen_input,
                         user_data="listen")

    def add_marks(self, scale, mark_current, list_degrees):
        set_degrees = set(list_degrees)
        for degree in set_degrees:
            scale.add_mark(degree, Gtk.PositionType.TOP, str(degree))

    def unlock_buttons(self):
        unlock = self.wheel_service.supported_wheel()
        self.ui_scale_test_wheel.set_sensitive(unlock)
        self.ui_scale_test_throttle.set_sensitive(unlock)
        self.ui_scale_test_brake.set_sensitive(unlock)
        self.ui_scale_test_clutch.set_sensitive(unlock)
        unlock_ff= unlock and (self.wheel_service.supported_features().get('ff_autocenter')
                                or self.wheel_service.supported_features().get('ff_gain'))
        self.test_rumble_button.set_sensitive(unlock_ff)

    def on_test_rumble_clicked(self,button):
        logger.info("on_test_rumble_clicked")
        if self.wheel_service.supported_wheel(force_validation=True):
            self.wheel_service.test_rumble()
        else:
            self.reload()


class FileChooserWindow(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="FileChooser")

    def add_filters(self, dialog):
        filter_text = Gtk.FileFilter()
        filter_text.set_name("JSON")
        filter_text.add_mime_type("application/json")
        dialog.add_filter(filter_text)

    def save_file_chooser(self, callback):
        dialog = Gtk.FileChooserDialog(_("Export profile"), self,
                                       Gtk.FileChooserAction.SAVE,
                                       (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                        Gtk.STOCK_SAVE, Gtk.ResponseType.OK))
        self.add_filters(dialog)
        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            logger.info("export confirmed")
            logger.info("File selected: " + dialog.get_filename())
            callback(dialog.get_filename())
        elif response == Gtk.ResponseType.CANCEL:
            logger.info("export canceled")
        dialog.destroy()

    def open_file_chooser(self, callback):
        dialog = Gtk.FileChooserDialog(_("Please choose a profile to import"), self,
                                       Gtk.FileChooserAction.OPEN,
                                       (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                        Gtk.STOCK_OPEN, Gtk.ResponseType.OK))
        self.add_filters(dialog)
        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            logger.info("import confirmed")
            logger.info("File selected: " + dialog.get_filename())
            callback(dialog.get_filename())
        elif response == Gtk.ResponseType.CANCEL:
            logger.info("import canceled")
        dialog.destroy()


class ProfileEditorUIHandler:

    def __init__(self, builder, preferences_service):
        self.builder = builder
        self.preferences_service = preferences_service
        self.ui_scale_range = self.builder.get_object("ui_scale_range_profile_editor")
        self.ui_scale_range.set_range(40, 900)
        self.ui_scale_ff_resistance = self.builder.get_object("ui_scale_ff_resistance_profile_editor")
        self.ui_scale_ff_gain = self.builder.get_object("ui_scale_ff_gain_profile_editor")
        self.button_save_profile_editor = self.builder.get_object("ui_gtkButton_save_profile_editor")
        self.button_new_profile_editor = self.builder.get_object("ui_gtkbutton_new_profile_editor")
        self.button_delete_profile_editor = self.builder.get_object("ui_gtkbutton_delete_profile_editor")
        self.button_export_profile_editor = self.builder.get_object("ui_gtkbutton_export_profile_editor")
        self.button_import_profile_editor = self.builder.get_object("ui_gtkbutton_import_profile_editor")
        self.description_profile_editor = self.builder.get_object("ui_gtktext_description_profile_editor")
        self.check_combine_pedals_profile_editor = self.builder.get_object('ui_gtkcheck_combine_pedals_profile_editor')
        self.alternate_modes_profile_editor = self.builder.get_object("ui_gtkComboBox_alternate_modes_profile_editor")
        self.author_profile_editor = self.builder.get_object("ui_gtkentry_author_profile_editor")
        self.tags_profile_editor = self.builder.get_object("ui_gtkEntry_tags_profile_editor")
        self.name_profile_editor = self.builder.get_object("ui_gtkentry_name_profile_editor")
        self.profile_profile_editor =  self.builder.get_object("ui_gtkComboBox_profile_profile_editor")
        self.profile_profile_editor.set_entry_text_column(0)
        self.connect_signals()

    def connect_signals(self):
        self.profile_profile_editor.connect("changed", self.on_profile_profile_editor_combo_changed)
        self.button_save_profile_editor.connect("clicked", self.on_save_profile_editor_clicked)
        self.button_new_profile_editor.connect("clicked", self.on_new_profile_editor_clicked)
        self.button_delete_profile_editor.connect("clicked", self.on_delete_profile_editor_clicked)
        self.button_export_profile_editor.connect("clicked", self.on_export_profile_editor_clicked)
        self.button_import_profile_editor.connect("clicked", self.on_import_profile_editor_clicked)

    def on_import_profile_editor_clicked(self, widget):
        file_chooser = FileChooserWindow()
        file_chooser.open_file_chooser(self.import_profile_editor)

    def import_profile_editor(self, path):
        logger.info("export "+path)
        profile_name = self.preferences_service.import_profile(path)
        self.reload(profile_name)

    def on_export_profile_editor_clicked(self, widget):
        file_chooser = FileChooserWindow()
        file_chooser.save_file_chooser(self.export)

    def export(self, path):
        logger.info("export "+path)
        name = self.name_profile_editor.get_text()
        self.preferences_service.export_profile(name, path,"json")

    def on_profile_profile_editor_combo_changed(self, combo):
        logger.info("on_profile_profile_editor_combo_changed")
        tree_iter = combo.get_active_iter()
        if tree_iter is not None:
            model = combo.get_model()
            profile = model[tree_iter][0]
            profile_name = next((item for item in self.list_profile_names()
                                         if item == profile), None)
            logger.info(profile_name)
            profile_loaded = self.preferences_service.read_profile(profile_name)
            self.populate(profile_loaded)

    def populate(self, profile):
        self.ui_scale_range.set_range(40, 900)
        self.name_profile_editor.set_text(profile.get("file_name",""))
        self.author_profile_editor.set_text(profile.get("author", ""))
        index = 0
        try:
            index = [idx for idx, s in enumerate(self.get_alternate_modes()) if profile["alternate_modes"] in s][0]
        except IndexError:
            index = 0
            logger.info("alternate_mode not found")
        self.populate_alternate_modes_combo(index)
        self.ui_scale_range.set_value(profile.get("range", 180))
        self.ui_scale_ff_resistance.set_value(profile.get("ff_autocenter", 0))
        self.ui_scale_ff_gain.set_value(profile.get("ff_gain", 100))
        if profile.get("combine_pedals", 0) > 0:
            self.check_combine_pedals_profile_editor.set_active(True)
        else:
            self.check_combine_pedals_profile_editor.set_active(False)
        list_tags = profile.get("tags", [])
        strtags =', '.join(list_tags)
        self.tags_profile_editor.set_text(strtags)
        self.description_profile_editor.get_buffer().set_text(profile.get("description", None))

    @staticmethod
    def get_alternate_modes():
        return ["G29: G29 Racing Wheel","G27: G27 Racing Wheel","G25: G25 Racing Wheel", "DFGT: Driving Force GT", "DFP: Driving Force Pro"]

    def populate_alternate_modes_combo(self, index=0):
        list_modes = self.get_alternate_modes()
        logger.info(list_modes)
        self.alternate_modes_profile_editor.set_entry_text_column(1)
        self.alternate_modes_profile_editor.set_model(self.get_combo_model_alternate_modes(self.get_alternate_modes()))
        self.alternate_modes_profile_editor.set_active(index)

    def load_window(self):
        self.reload()

    def on_delete_profile_editor_clicked(self, button):
        selected = self.get_combo_box_text_selected(self.profile_profile_editor)
        try:
            index = self.list_profile_names(list_default=False).index(selected)
        except ValueError as v:
            logger.warning(v)
            index = 0
        try:
            self.delete_profile(selected)
            self.profile_profile_editor.set_entry_text_column(0)
            self.profile_profile_editor.set_model(self.get_combo_model_profile(self.list_profile_names(list_default=False)))
            if index > 0:
                self.profile_profile_editor.set_active(index - 1)
                self.on_profile_profile_editor_combo_changed(self.profile_profile_editor)
            else:
                if len(self.list_profile_names(list_default=False))  > 0:
                    self.profile_profile_editor.set_active(index)
                    self.on_profile_profile_editor_combo_changed(self.profile_profile_editor)
                else:
                    self.new_profile()

        except OSError as os:
            logger.error(os)

    def delete_profile(self, profile_name):
        if profile_name != "default":
            self.preferences_service.delete_profile(profile_name)
        else:
            logger.info("You can not delete the default.profile")

    def on_new_profile_editor_clicked(self, button):
        self.new_profile()

    def on_save_profile_editor_clicked(self,button):
        logger.info("on_save_profile_editor_clicked")
        combine_pedals = 0
        if self.check_combine_pedals_profile_editor.get_active():
            combine_pedals = 1
        selected_mode = self.get_combo_box_text_selected(self.alternate_modes_profile_editor)
        tags_list = []
        if self.tags_profile_editor.get_text():
            tags_list = [tag.strip() for tag in self.tags_profile_editor.get_text().split(",")]
        buffer = self.description_profile_editor.get_buffer()
        profile = {"file_name": self.name_profile_editor.get_text(), "version": UtilPath.version, "range": self.ui_scale_range.get_value(),
                       "combine_pedals": combine_pedals, "alternate_modes": selected_mode, 'ff_gain': self.ui_scale_ff_gain.get_value(),
                       'ff_autocenter': self.ui_scale_ff_resistance.get_value(), "author":self.author_profile_editor.get_text(),
                   "tags": tags_list, "description":buffer.get_text(buffer.get_start_iter(), buffer.get_end_iter(), False)}
        self.preferences_service.write_profile(profile)
        self.reload(self.name_profile_editor.get_text())

    def get_combo_box_text_selected(self, combo):
        selected = None
        tree_iter = combo.get_active_iter()
        if tree_iter is not None:
            model = combo.get_model()
            selected = model.get_value(tree_iter,0)
        else:
            entry = combo.get_child()
            selected = str(entry.get_text())
        return selected

    def list_profile_names(self,list_default=True):
        list_prf = self.preferences_service.list_profiles()
        lst = []
        for item in list_prf:
            if list_default:
                lst.append(item.replace(".json",""))
            else:
                if "default.json" == item:
                    logger.info("list default is not going to added to the list")
                else:
                    lst.append(item.replace(".json", ""))
        return lst

    def new_profile(self):
        profile = {"file_name": "new", "version": UtilPath.version, "range": 180, "combine_pedals": 0,
                   "alternate_modes": "G29", "ff_gain": 100, "ff_autocenter": 0, "author": getpass.getuser(),
                   "tags": [], "description": ""}
        self.populate(profile)

    def reload(self, filter=None):
        range_mark_degrees = [0,180, 270, 360, 540, 720,900]
        ff_mark_degrees = [0, 25, 50, 75, 100]
        self.ui_scale_range.clear_marks()
        self.ui_scale_ff_resistance.clear_marks()
        self.ui_scale_ff_gain.clear_marks()
        self.add_marks(self.ui_scale_range, range_mark_degrees)
        self.add_marks(self.ui_scale_ff_resistance, ff_mark_degrees)
        self.add_marks(self.ui_scale_ff_gain, ff_mark_degrees)
        list_prf = self.preferences_service.list_profiles()
        if list_prf is not None and len(list_prf) > 0:
            try:
                list_prf.remove("default.json")
            except ValueError as vr:
                logger.error(vr)
        if list_prf is not None and len(list_prf) > 0:
            self.profile_profile_editor.set_entry_text_column(0)
            self.profile_profile_editor.set_model(self.get_combo_model_profile(self.list_profile_names(list_default=False)))
            try:
                index = self.list_profile_names(list_default=False).index(filter)
            except ValueError as v:
                logger.warning(v)
                index = 0
            self.profile_profile_editor.set_active(index)
            self.on_profile_profile_editor_combo_changed(self.profile_profile_editor)
        else:
            self.new_profile()

    def get_combo_model_profile(self, param_list):
        list_store = Gtk.ListStore(str)
        if param_list is not None:
            for item in param_list:
                logger.info(item)
                list_store.append([item])
        return list_store

    def get_combo_model_alternate_modes(self, param_list):
        list_store = Gtk.ListStore(str,str)
        if param_list is not None:
            for item in param_list:
                logger.info(item)
                temp = item.split(":")
                list_store.append([temp[0], temp[1]])
        return list_store

    def add_marks(self, scale, list_degrees):
        set_degrees = set(list_degrees)
        for degree in set_degrees:
            scale.add_mark(degree, Gtk.PositionType.TOP,str(degree))


class WheelUIHandler:

    def __init__(self, builder, wheel_service, preferences_service):
        self.builder = builder
        self.wheel_service = wheel_service
        self.preferences_service = preferences_service
        self.check_combine_pedals = None
        self.save_button = None
        self.label_wheel = None
        self.label_wheel = self.builder.get_object("ui_label_wheel")
        self.scale_range = self.builder.get_object("ui_scale_range")
        self.scale_autocenter = self.builder.get_object("ui_scale_autocenter")
        self.scale_gain = self.builder.get_object("ui_scale_gain")
        self.save_button = self.builder.get_object("ui_save_button")
        self.reload_button = self.builder.get_object("ui_reload_button")
        self.advanced_options= self.builder.get_object("ui_gtkexpander_advanced_options")
        self.ff_options = self.builder.get_object("ui_gtk_expander_ff")
        self.check_combine_pedals = self.builder.get_object('ui_check_combine_pedals')
        self.alternate_modes_combo = self.builder.get_object("ui_alternate_modes_combo")
        self.profile_combo_wheel_editor = self.builder.get_object("ui_gtkcombo_profile_wheel_editor")
        self.current_mode = None
        self.connect_signals()

    def load_window(self):
        self.reload()


    def connect_signals(self):
        self.save_button.connect("clicked", self.onSavePressed)
        self.scale_range.connect("value-changed", self.scaleMoved)
        self.reload_button.connect("clicked", self.on_reload_button_clicked)
        self.profile_combo_wheel_editor.connect("changed", self.on_profile_combo_changed)

    def on_profile_combo_changed(self, combo):
        logger.info("on_profile_combo_changed")
        tree_iter = combo.get_active_iter()
        if tree_iter is not None:
            model = combo.get_model()
            profile = model[tree_iter][0]
            profile_name = next((item for item in self.list_profile_names(False)
                                         if item == profile), None)
            logger.info(profile_name)
            if profile_name is None or profile_name is not None and profile_name == " ":
                profile_name = "default"
                preference= self.preferences_service.read_preferences(UtilPath.default_preferences)
                load_last_value=preference.get("load_last_value",False)
                if load_last_value and  self.preferences_service.check_profile(UtilPath.default_profile):
                    profile_loaded = self.preferences_service.read_profile(profile_name)
                    self.populate(profile_loaded)
                else:
                    self.scale_range.set_value(self.wheel_service.current_degrees())
                    self.scale_autocenter.set_value(self.wheel_service.get_autocenter())
                    self.scale_gain.set_value(self.wheel_service.get_gain())
                    self.check_combine_pedals.set_active(self.wheel_service.get_combine_pedals())
                    self.populate_alternate_modes_combo()
            else:
                profile_loaded = self.preferences_service.read_profile(profile_name)
                self.populate(profile_loaded)

    def get_combo_model_profile(self, param_list):
        param_list.insert(0, " ")
        list_store = Gtk.ListStore(str)
        if param_list is not None:
            for item in param_list:
                logger.info(item)
                list_store.append([item])
        return list_store

    def populate_combo_profile(self):
        list_prf = self.preferences_service.list_profiles()
        if list_prf is not None and len(list_prf) > 0:
            self.profile_combo_wheel_editor.set_entry_text_column(0)
            self.profile_combo_wheel_editor.set_model(self.get_combo_model_profile(self.list_profile_names(list_default=False)))
            self.profile_combo_wheel_editor.set_active(0)

    def populate(self, profile):
        if profile.get("range", 180) > self.wheel_service.max_degrees:
            self.scale_range.set_value(self.wheel_service.max_degrees)
        elif profile.get("range", 180) < self.wheel_service.min_degrees:
            self.scale_range.set_value(self.wheel_service.min_degrees)
        else:
            self.scale_range.set_value(profile.get("range", 180))
        self.scale_autocenter.set_value(profile.get("ff_autocenter", 0))
        self.scale_gain.set_value(profile.get("ff_gain", 100))
        if profile.get("combine_pedals", 0) > 0:
            self.check_combine_pedals.set_active(True)
        else:
            self.check_combine_pedals.set_active(False)
        list_modes = self.wheel_service.get_alternate_modes()
        logger.info(list_modes)
        mode = profile.get("alternate_modes", "G29")
        self.alternate_modes_combo.set_active(self.get_select_id_modes(list_modes,mode))

    def get_select_id_modes(self, param_list, mode):
        if param_list is not None:
            filtered_list = [x for x in param_list if "DF-EX" not in x and "native" not in x]
            indices = [i for i, s in enumerate(filtered_list) if mode in s]
            logger.info(indices)
            if indices is not None and len(indices)> 0:
                return indices[0]
            else:
                return self.get_initial_id_modes(param_list)
        else:
            return 0

    def list_profile_names(self,list_default=True):
        list_prf = self.preferences_service.list_profiles()
        lst = []
        for item in list_prf:
            if list_default:
                lst.append(item.replace(".json",""))
            else:
                if "default.json" == item:
                    logger.info("list default is not going to added to the list")
                else:
                    lst.append(item.replace(".json", ""))
        return lst

    def on_mainWindow_destroy(self, *args):
        Gtk.main_quit()

    def onDestroy(self, *args):
        Gtk.main_quit()

    def onSavePressed(self, button):
        logger.info("onSavePressed")
        if self.wheel_service.supported_wheel(force_validation=True):
            logger.info(self.scale_range.get_value())
            logger.info(self.check_combine_pedals.get_active())
            logger.info("alternate mode")
            selected_mode = self.get_combo_box_text_selected(self.alternate_modes_combo)
            logger.info(selected_mode)
            selected_mode_temp = selected_mode.split(":")
            try:
                self.wheel_service.write_alternate_modes(selected_mode_temp[0])
                self.wheel_service.write_driver_degrees(self.scale_range.get_value())
                self.wheel_service.write_driver_combine_pedals(self.check_combine_pedals.get_active())
                logger.info("write autocenter")
                logger.info(self.scale_autocenter.get_value())
                self.wheel_service.write_autocenter(self.scale_autocenter.get_value())
                logger.info("write gain")
                logger.info(self.scale_gain.get_value())
                self.wheel_service.write_gain(self.scale_gain.get_value())
                self.wheel_service.synchronize_from_driver_to_profile(UtilPath.default_profile)
                self.populate_alternate_modes_combo()
            except ValueError as v:
                logger.error(v)
                self.reload()
        else:
            self.reload()

    def on_combine_pedals_clicked(self, check):
        logger.info("combine_pedals_clicked")
        logger.info(check.get_active())

    def on_reload_button_clicked(self,button):
        logger.info("on_reload_button_clicked")
        self.reload()

    def scaleMoved(self, scale):
        logger.info(scale.get_value())

    def add_marks(self, scale, mark_current, list_degrees):
        set_degrees = set(list_degrees)
        for degree in set_degrees:
            scale.add_mark(degree, Gtk.PositionType.TOP,str(degree))

    def get_combo_model(self, param_list):
        list_store = Gtk.ListStore(str,str)
        if param_list is not None:
            for item in param_list:
                logger.info(item)
                temp = item.split(":")
                if "native" not in temp[0] and "DF-EX" not in temp[0]:
                    list_store.append([temp[0], temp[1]])
        return list_store

    def get_initial_id_modes(self, param_list):
        if param_list is not None:
            filtered_list = [x for x in param_list if "DF-EX" not in x and "native" not in x]
            indices = [i for i, s in enumerate(filtered_list) if '*' in s]
            logger.info(indices)
            if indices is not None and len(indices)>0:
                return indices[0]
            else:
                return 0
        else:
            return 0

    def get_combo_box_text_selected(self, combo):
        selected = None
        tree_iter = combo.get_active_iter()
        if tree_iter is not None:
            model = combo.get_model()
            selected = model.get_value(tree_iter,0)
        else:
            entry = combo.get_child()
            selected = str(entry.get_text())
        return selected

    def populate_alternate_modes_combo(self):
        logger.info("populate_alternate_modes_combo")
        list_modes = self.wheel_service.get_alternate_modes()
        logger.info(list_modes)
        self.alternate_modes_combo.set_entry_text_column(1)
        self.alternate_modes_combo.set_model(self.get_combo_model(list_modes))
        self.alternate_modes_combo.set_active(self.get_initial_id_modes(list_modes))

    def unlock_buttons(self):
        unlock = self.wheel_service.supported_wheel()
        self.save_button.set_sensitive(unlock)
        self.scale_range.set_sensitive(unlock)
        self.scale_autocenter.set_sensitive(unlock)
        self.scale_gain.set_sensitive(unlock)
        self.profile_combo_wheel_editor.set_sensitive(unlock)
        unlock_combine_pedals = unlock and self.wheel_service.supported_features().get('combine_pedals')
        self.check_combine_pedals.set_sensitive(unlock_combine_pedals)
        unlock_alternate_modes = unlock and self.wheel_service.supported_features().get('alternate_modes')
        self.alternate_modes_combo.set_sensitive(unlock_alternate_modes)
        unlock_ff=  unlock and (self.wheel_service.supported_features().get('ff_autocenter')
                                or self.wheel_service.supported_features().get('ff_gain'))
        self.ff_options.set_sensitive(unlock_ff)
        advanced_options_check = unlock_combine_pedals or unlock_alternate_modes
        self.advanced_options.set_sensitive(advanced_options_check)
        self.ff_options.set_expanded(unlock_ff)
        self.advanced_options.set_expanded(advanced_options_check)

    def reload(self):
        self.wheel_service = ScanService().get_wheel()
        preference = self.preferences_service.read_preferences(UtilPath.default_preferences)
        if preference["load_last_value"] and self.preferences_service.check_profile(UtilPath.default_profile):
            logger.info("profile default exists and load_last_value propertie is activated")
            self.wheel_service.synchronize_from_profile_to_driver(UtilPath.default_profile)
        self.label_wheel.set_text(str(self.wheel_service.name_wheel()))
        self.populate_alternate_modes_combo()
        self.scale_range.set_range(self.wheel_service.min_degrees, self.wheel_service.max_degrees)
        self.scale_range.set_value(self.wheel_service.current_degrees())
        self.scale_range.clear_marks()
        default_mark_degrees =[180, 270, 360, 540, 720]
        list_degrees = [x for x in default_mark_degrees if x > self.wheel_service.min_degrees  and x < self.wheel_service.max_degrees]
        list_degrees.extend([self.wheel_service.max_degrees, self.wheel_service.min_degrees])
        self.add_marks(self.scale_range, self.wheel_service.current_degrees(), list_degrees)
        self.scale_autocenter.set_range(0,100)
        self.scale_autocenter.set_value(self.wheel_service.get_autocenter())
        self.scale_autocenter.clear_marks()
        list_autocenter_degrees = [x for x in range(101) if x % 20 == 0]
        list_autocenter_degrees.extend([25,50,75])
        self.add_marks(self.scale_autocenter ,0,list_autocenter_degrees)
        self.scale_gain.set_range(0, 100)
        self.scale_gain.set_value(self.wheel_service.get_gain())
        self.scale_gain.clear_marks()
        self.add_marks(self.scale_gain, 100, list_autocenter_degrees)
        self.check_combine_pedals.set_active(self.wheel_service.get_combine_pedals())
        self.populate_combo_profile()
        self.unlock_buttons()


class MainUI:

    def __init__(self, builder, wheel_service, util_path, preferences_service):
        self.builder = builder
        self.window = None
        self.preferences_service = preferences_service
        self.util_path = util_path
        self.glade_file = os.path.join(util_path.get_glade_files_path(), "mainui.glade")
        self.builder.add_from_file(self.glade_file)
        self.preference_ui_handler = PreferencesUIHandler(self.builder, self.preferences_service)
        self.wheel_ui_handler = WheelUIHandler(self.builder, wheel_service, self.preferences_service)
        self.test_ui_handler = TestUIHandler(self.builder, wheel_service, self.preferences_service)
        self.profile_editor_ui_handler = ProfileEditorUIHandler(self.builder, self.preferences_service)
        self.builder.get_object("ui_notebook").connect("switch_page", self.on_switch_page)

    def on_switch_page(self, notebook, tab, index):
        logger.info("on_switch_page")
        if index == 1:
            self.test_ui_handler.reload()
        elif index == 0:
            self.wheel_ui_handler.reload()

    def load_window(self):
        self.wheel_ui_handler.load_window()
        self.preference_ui_handler.load_window()
        self.test_ui_handler.load_window()
        self.profile_editor_ui_handler.load_window()
        self.window = self.builder.get_object("main_window")
        self.window.connect('destroy', Gtk.main_quit)
        self.window.show_all()
        Gtk.main()


if __name__ == '__main__':
    if getattr(sys, '_MEIPASS', ''):
        print('running pyLinuxWheel as PyInstaller bundle')
    else:
        print('running in a normal Python process')
    util_path = UtilPath()
    logger.info("pyLinuxWheel version %s" % util_path.version)
    logger.info("locale directory:" + util_path.get_po_path())
    LOCALE_DIR = util_path.get_po_path()
    locale.setlocale(locale.LC_ALL, '')
    locale.bindtextdomain(APP, LOCALE_DIR)
    gettext.bindtextdomain(APP, LOCALE_DIR)
    gettext.textdomain(APP)
    _ = gettext.gettext
    app = App()
    app.execute()
