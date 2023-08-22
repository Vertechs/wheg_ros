import numpy as np


class CPG:
    def euler_update(self,t_step):
        """
        Calculate new states and derivatives, update states according to Eulers method
        :param t_step: time step to update over
        """
        raise NotImplementedError

    def graph_output(self):
        """
        :return: array of floats with size = (N_oscillator,1)
        :rtype: np.ndarray
        """
        raise NotImplementedError

    def perturbation(self, n, sigmas):
        """
        Apply a random perturbation to the states of an oscillator
        :param sigmas: array of std_deviation for each state
        :param n: oscillator to apply the perturbation to
        """
        raise NotImplementedError

    def reset_oscillators(self):
        """
        Reset the states of oscillators in the network
        :param n: list of oscillators to reset, n=-1 to reset all
        :return:
        """
        raise NotImplementedError

    def diff_input(self,v : float, w : float, h : float):
        """
        Apply commanded velocities and extensions to CPG parameters
        :param v: desired forward velocity of robot
        :param w: desired angular velocity of robot
        :param h: desired ride height in same units as radius
        :return:
        """
        raise NotImplementedError

    def wheel_output(self):
        """
        Send wheel position commands as lists of rotation and extension values
        :return: rot,ext
        :rtype rot: list
        :rtype ext: list
        """
        raise NotImplementedError

    def wheel_feedback(self,rot,ext):
        """
        Update internal states or apply feedback signal using wheel position estimates passed up from controller
        :param rot: array or list of phase position values for each wheel
        :param ext: array or list of extension amounts for each wheel
        :return:
        """
        raise NotImplementedError







