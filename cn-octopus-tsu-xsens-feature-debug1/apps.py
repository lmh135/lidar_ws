from octopus_console.utils.usb_detector import find_usb_device

def generate_apps(vehicle, mode):
    xsens = dict()
    xsens = [vehicle.get_component('xsens')]
    if not xsens:
        return {}

    if xsens[0]:
        xsens_profile = {'cmd': 'zoro launch device_xsens xsens_driver.launch',
                         'module': 'sensor',
                         'out_topics': ['/xsens_driver/imu/data',
                                    '/xsens_driver/imu/mag',
                                    '/xsens_driver/imu_data_str',
                                    '/xsens_driver/imupos',
                                    '/xsens_driver/velocity'],
                         'args': {'dev_name': xsens[0].device,
                                  'baudrate': '460800',
                                 }
                        }
        return {'xsens': xsens_profile}
