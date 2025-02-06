import time
import sys
import numpy as np
import pinocchio as pin    
from pinocchio import casadi as cpin                
from pinocchio.robot_wrapper import RobotWrapper    
from pinocchio.visualize import MeshcatVisualizer 

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

from robot_descriptions.loaders.pinocchio import load_robot_description

kNumMotors = 20

init_joint = np.array([
    0.0, -0.2, 0.5, 0.0, -0.2, 0.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0
],
dtype=float) # 19+1

motor_cmd = np.array([
    0.0, -0.2, 0.5, 0.0, -0.2, 0.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0
],
dtype=float) # 20

q = np.array([])
tau = np.array([])

#######################
class Robot_IK:
    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        # self.robot = pin.RobotWrapper.BuildFromURDF('../assets/h1_description/urdf/h1_with_hand.urdf', '../assets/h1_description/urdf')
        # self.robot = pin.RobotWrapper.BuildFromURDF('/robot_description/urdf/h1_with_hand.urdf', '/robot_description/urdf') # for test

        self.robot = load_robot_description("h1_mj_description")

        self.model = self.robot.model
        self.data = self.robot.data

        frame_id = self.model.getFrameId("right_ankle_link") 

        print("dof: ",self.model.nv)

        self.q0 = np.zeros(kNumMotors+6) # seems dof is 25, but coordinate is 26 !
        self.q0[6:] = init_joint
            
        # q_ref = pin.integrate(self.model, q0, 0.03* np.random.rand(self.model.nv))

        # self.robot.display(q0)

        self.v0 = np.zeros(self.model.nv)
        self.v_ref = self.v0.copy()

        self.a0 = np.zeros(self.model.nv)
    
        self.data_sim = self.model.createData()
        self.data_control = self.model.createData()


        contact_models = []
        contact_datas = [] 

        frame = self.model.frames[frame_id]

        self.contact_model = pin.RigidConstraintModel(
                pin.ContactType.CONTACT_6D, self.model, frame.parentJoint, frame.placement
        )

        contact_models.append(self.contact_model)
        contact_datas.append(self.contact_model.createData())


        num_constraints=1
        self.contact_dim = 6 * num_constraints

        pin.initConstraintDynamics(self.model, self.data_sim, contact_models)

        self.q = self.q0.copy()
        self.v = self.v0.copy()
        self.tau = np.zeros(self.model.nv)

    def ik_func(self):
        # q = self.q0.copy()
        # return q
        J_constraint = np.zeros((self.contact_dim, self.model.nv))
        pin.computeJointJacobians(self.model, self.data_control, self.q)

        J_constraint[ :6, :] = pin.getFrameJacobian(
                self.model,
                self.data_control,
                self.contact_model.joint1_id,
                self.contact_model.joint1_placement,
                self.contact_model.reference_frame,
        )

        # A = np.vstack((S, J_constraint))
        # b = pin.rnea(model, data_control, q, v, np.zeros(model.nv))

        self.g_grav = pin.rnea(self.model, self.data, self.q0, self.v0, self.a0) # 25

        sol = np.linalg.lstsq(J_constraint.T, self.g_grav, rcond=None)[0]

        self.tau = np.concatenate((np.zeros((6)), sol[: self.model.nv - 6]))


        # self.opti.set_value(self.param_tf_l, left_pose)
        # self.opti.set_value(self.param_tf_r, right_pose)

        return self.g_grav

#######################

dt = 0.002
runing_time = 0.0
crc = CRC()

low_state = None  
def LowStateMessageHandler(msg: LowState_):
        low_state = msg

input("Press enter to start")

if __name__ == '__main__':
    h1_ik = Robot_IK()

    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    pub.Init()
    sub.Init(LowStateMessageHandler, 10)

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(kNumMotors):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        tau = h1_ik.ik_func()
        
        # motor - joint transform
        for i in range(9):
            motor_cmd[i] = tau[i]
        for i in range(10):
            motor_cmd[i+9] = tau[i+10]

        print(motor_cmd)


        # Total time for standing up or standing down is about 1.2s
        for i in range(kNumMotors):
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = motor_cmd[i]

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
