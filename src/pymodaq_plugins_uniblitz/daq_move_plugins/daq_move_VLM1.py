from typing import Union, List, Dict
from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)
from pymodaq_utils.utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq_gui.parameter import Parameter

from serial import Serial
from serial.tools import list_ports


class DAQ_Move_VLM1(DAQ_Move_base):
    """ Instrument plugin class for VLM1 Shutter Controller.
    
    It uses the serial module to send one of two simple commands. Doesn't require any special driver.
    Position 0 is closed, 1 is open. Any move command to a nonzero position will default to 1.

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library.

    """
    is_multiaxes = False
    _axis_names: Union[List[str], Dict[str, int]] = ['Status']
    _controller_units: Union[str, List[str]] = ''
    _epsilon: Union[float, List[float]] = 0.1
    data_actuator_type = DataActuatorType.DataActuator

    COMports = [COMport.device for COMport in list_ports.comports()]

    if len(COMports) > 0:
        if 'COM10' in COMports:
            COMport = 'COM10'
        else:
            COMport = COMports[0]
    else:
        COMport = None

    params = [{'title': 'COM Port:', 'name': 'COM_port', 'type': 'list', 'limits': COMports, 'value': COMport},
                ] + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    def ini_attributes(self):
        self.controller: Serial = None

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = self.current_value
        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Terminate the communication protocol"""
        if self.controller is not None:
            self.controller.close()

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == "COM_Port":
            self.close()
            self.controller = Serial(param.value(), baudrate=9600)
        else:
            pass

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        self.ini_stage_init(slave_controller=controller)  # will be useful when controller is slave

        if self.is_master:  # is needed when controller is master
            self.controller = Serial(self.settings.child('COM_port').value(), baudrate=9600)

        info = "Shutter connected on port "+str(self.settings.child('COM_port').value())
        initialized = True
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one

        int_value = int(value > 0)
        self.controller.write([b'A', b'@'][int_value])
        self.current_value = int_value

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        self.move_abs(DataActuator(data=int(value.value() > 0)))

    def move_home(self):
        """Call the reference method of the controller"""
        self.move_abs(DataActuator(data=0))
        self.current_value = 0

    def stop_motion(self):
        pass

if __name__ == '__main__':
    main(__file__)
