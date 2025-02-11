import time
import sys
import numpy as np
import pinocchio as pin    
# from pinocchio import casadi as cpin                
from pinocchio.robot_wrapper import RobotWrapper    
from pinocchio.visualize import MeshcatVisualizer 

from enum import IntEnum

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

from robot_descriptions.loaders.pinocchio import load_robot_description

kNumMotors = 20

class MotorJntIndex(IntEnum):
    # Right leg
    RightHipYaw = 8
    RightHipRoll = 0
    RightHipPitch = 1
    RightKnee = 2
    RightAnkle = 11
    # Left leg
    LeftHipYaw = 7
    LeftHipRoll = 3
    LeftHipPitch = 4
    LeftKnee = 5
    LeftAnkle = 10

    WaistYaw = 6

    NotUsedJoint = 9

    # Right arm
    RightShoulderPitch = 12
    RightShoulderRoll = 13
    RightShoulderYaw = 14
    RightElbow = 15
    # Left arm
    LeftShoulderPitch = 16
    LeftShoulderRoll = 17
    LeftShoulderYaw = 18
    LeftElbow = 19

class MJModelIndex(IntEnum):
    # Right leg
    RightHipYaw = 5
    RightHipRoll = 6
    RightHipPitch = 7
    RightKnee = 8
    RightAnkle = 9
    # Left leg
    LeftHipYaw = 0
    LeftHipRoll = 1
    LeftHipPitch = 2
    LeftKnee = 3
    LeftAnkle = 4

    WaistYaw = 10

    # Right arm
    RightShoulderPitch = 15
    RightShoulderRoll = 16
    RightShoulderYaw = 17
    RightElbow = 18
    # Left arm
    LeftShoulderPitch = 11
    LeftShoulderRoll = 12
    LeftShoulderYaw = 13
    LeftElbow = 14


init_joint = np.array([
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0
],
dtype=float) # 19

motor_cmd = np.array([
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0
],
dtype=float) # 20

tau_ref = np.array([
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0
],
dtype=float) # 19

stand_down_joint_pos = np.array([
    0.0, -0.8, 1.0, 0.0, -0.8, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0
],
dtype=float)

q = np.array([])
tau = np.array([])

#######################
class Robot_IK:
    def __init__(self):
        self.robot = load_robot_description("h1_mj_description")

        self.model = self.robot.model
        self.data = self.robot.data

        self.frame_id = self.model.getFrameId("right_ankle_link") 
        # self.frame_id = 23

        print("dof: ",self.model.nv)
        print("frame_id: ",self.frame_id)

        self.q0 = np.zeros(kNumMotors+6) # seems dof is 25, but coordinate is 26 !
        self.q0[7:] = init_joint
        self.q0[6] = 1
            
        # q_ref = pin.integrate(self.model, q0, 0.03* np.random.rand(self.model.nv))

        self.v0 = np.zeros(self.model.nv)
        self.a0 = np.zeros(self.model.nv)
    
        # self.data_sim = self.model.createData()
        # self.data_control = self.model.createData()

        self.robot.setVisualizer(MeshcatVisualizer())
        self.robot.initViewer(open=True)
        self.robot.loadViewerModel("pinocchio")
        self.robot.display(self.q0)

        print("Init. state q:", self.q0)
        # breakpoint()

        self.contact_models = []
        contact_datas = [] 

        frame = self.model.frames[self.frame_id]

        # get the id of the body link (pelvis)
        self.bl_id = self. model.getFrameId("pelvis")

        self.contact_model = pin.RigidConstraintModel(
                pin.ContactType.CONTACT_6D, self.model, frame.parentJoint, frame.placement
        )

        self.contact_models.append(self.contact_model)
        contact_datas.append(self.contact_model.createData())


        self.num_constraints = 1
        self.contact_dim = 6 * self.num_constraints

        # pin.initConstraintDynamics(self.model, self.data_sim, contact_models)

        self.q = self.q0.copy()
        self.v = self.v0.copy()
        self.tau = np.zeros(self.model.nv - 6)

    def ik_func(self, q0):
        self.q0 = q0

        pin.framesForwardKinematics(self.model, self.data, self.q0)
        pin.updateFramePlacements(self.model, self.data)

        self.g_grav = pin.rnea(self.model, self.data, self.q0, self.v0, self.a0) # 25

        print("grav. vec.: ",self.g_grav)

        g_bl = self.g_grav[:6]
        g_j = self.g_grav[6:]


        Js__foot_q = np.copy(pin.computeFrameJacobian(self.model, self.data, self.q0, self.frame_id, pin.WORLD))

        # get the jacobian between contact foot and body linktau
        Js__foot_bl = np.copy(Js__foot_q[:6, :6]) 
        Js__foot_bj = np.copy(Js__foot_q[:, 6:])

        G_up = np.linalg.pinv(Js__foot_bl) @ Js__foot_bj
        mat_E = np.identity(19)
        mat_G = np.zeros((25, 19))
        mat_G[:6, :] = G_up
        mat_G[6:, :] = mat_E

        # print("Contact Jacobian in world frame 1: ", Js__foot_bl)
        # print("Contact Jacobian in world frame 2: ", Js__foot_bj)

        print("G1: ", mat_G)
        G_T = mat_G.transpose()

        self.tau = G_T @ self.g_grav
        # self.tau = self.g_grav[6:]

        return self.tau

    
    def joint_torq(self, tau):
        motor_tau = np.zeros(kNumMotors)

        motor_tau[MotorJntIndex.LeftAnkle.value] = tau[MJModelIndex.LeftAnkle.value]
        motor_tau[MotorJntIndex.LeftKnee.value] = tau[MJModelIndex.LeftKnee.value]
        motor_tau[MotorJntIndex.LeftHipPitch.value] = tau[MJModelIndex.LeftHipPitch.value]
        motor_tau[MotorJntIndex.LeftHipRoll.value] = tau[MJModelIndex.LeftHipRoll.value]
        motor_tau[MotorJntIndex.LeftHipYaw.value] = tau[MJModelIndex.LeftHipYaw.value]

        motor_tau[MotorJntIndex.RightAnkle.value] = tau[MJModelIndex.RightAnkle.value]
        motor_tau[MotorJntIndex.RightKnee.value] = tau[MJModelIndex.RightKnee.value]
        motor_tau[MotorJntIndex.RightHipPitch.value] = tau[MJModelIndex.RightHipPitch.value]
        motor_tau[MotorJntIndex.RightHipRoll.value] = tau[MJModelIndex.RightHipRoll.value]
        motor_tau[MotorJntIndex.RightHipYaw.value] = tau[MJModelIndex.RightHipYaw.value]

        motor_tau[MotorJntIndex.WaistYaw.value] = tau[MJModelIndex.WaistYaw.value]

        motor_tau[MotorJntIndex.LeftShoulderPitch.value] = tau[MJModelIndex.LeftShoulderPitch.value]
        motor_tau[MotorJntIndex.LeftShoulderRoll.value] = tau[MJModelIndex.LeftShoulderRoll.value]
        motor_tau[MotorJntIndex.LeftShoulderYaw.value] = tau[MJModelIndex.LeftShoulderYaw.value]
        motor_tau[MotorJntIndex.LeftElbow.value] = tau[MJModelIndex.LeftElbow.value]

        motor_tau[MotorJntIndex.RightShoulderPitch.value] = tau[MJModelIndex.RightShoulderPitch.value]
        motor_tau[MotorJntIndex.RightShoulderRoll.value] = tau[MJModelIndex.RightShoulderRoll.value]
        motor_tau[MotorJntIndex.RightShoulderYaw.value] = tau[MJModelIndex.RightShoulderYaw.value]
        motor_tau[MotorJntIndex.RightElbow.value] = tau[MJModelIndex.RightElbow.value]

        return motor_tau
#######################

dt = 0.002
runing_time = 0.0
crc = CRC()

class State: 
    def __init__(self):
        self.low_state = None  
        self.q = np.zeros(kNumMotors+6)
        # self.q[6] = 1
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateMessageHandler, 10)

    def __ModelStateTrans(self, motor_state, quaternion):

        model_state = np.zeros(kNumMotors+6)

        # model_state[3:7] = quaternion

        # model_state[5] = quaternion[0]
        # model_state[3] = quaternion[1]
        # model_state[4] = quaternion[2]
        # model_state[6] = quaternion[3]

        model_state[6] = 1 #

        model_state[MJModelIndex.LeftAnkle.value+7] = motor_state[MotorJntIndex.LeftAnkle.value]
        model_state[MJModelIndex.LeftKnee.value+7] = motor_state[MotorJntIndex.LeftKnee.value]
        model_state[MJModelIndex.LeftHipPitch.value+7] = motor_state[MotorJntIndex.LeftHipPitch.value]
        model_state[MJModelIndex.LeftHipRoll.value+7] = motor_state[MotorJntIndex.LeftHipRoll.value]
        model_state[MJModelIndex.LeftHipYaw.value+7] = motor_state[MotorJntIndex.LeftHipYaw.value]

        model_state[MJModelIndex.RightAnkle.value+7] = motor_state[MotorJntIndex.RightAnkle.value]
        model_state[MJModelIndex.RightKnee.value+7] = motor_state[MotorJntIndex.RightKnee.value]
        model_state[MJModelIndex.RightHipPitch.value+7] = motor_state[MotorJntIndex.RightHipPitch.value]
        model_state[MJModelIndex.RightHipRoll.value+7] = motor_state[MotorJntIndex.RightHipRoll.value]
        model_state[MJModelIndex.RightHipYaw.value+7] = motor_state[MotorJntIndex.RightHipYaw.value]

        model_state[MJModelIndex.WaistYaw.value+7] = motor_state[MotorJntIndex.WaistYaw.value]

        model_state[MJModelIndex.LeftShoulderPitch.value+7] = motor_state[MotorJntIndex.LeftShoulderPitch.value]
        model_state[MJModelIndex.LeftShoulderRoll.value+7] = motor_state[MotorJntIndex.LeftShoulderRoll.value]
        model_state[MJModelIndex.LeftShoulderYaw.value+7] = motor_state[MotorJntIndex.LeftShoulderYaw.value]
        model_state[MJModelIndex.LeftElbow.value+7] = motor_state[MotorJntIndex.LeftElbow.value]

        model_state[MJModelIndex.RightShoulderPitch.value+7] = motor_state[MotorJntIndex.RightShoulderPitch.value]
        model_state[MJModelIndex.RightShoulderRoll.value+7] = motor_state[MotorJntIndex.RightShoulderRoll.value]
        model_state[MJModelIndex.RightShoulderYaw.value+7] = motor_state[MotorJntIndex.RightShoulderYaw.value]
        model_state[MJModelIndex.RightElbow.value+7] = motor_state[MotorJntIndex.RightElbow.value]

        # print("motor state: ", motor_state)

        # breakpoint()

        return model_state

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        # print(msg.motor_state[20].q)

        motor_state = np.zeros(kNumMotors)
        motor_torq = np.zeros(kNumMotors)
        for i in range(kNumMotors):
            motor_state[i] = msg.motor_state[i].q
            motor_torq[i] = msg.motor_state[i].tau_est

        self.q = self.__ModelStateTrans(motor_state, msg.imu_state.quaternion)


        # print("Joint state q:", self.q)
        # print("Quaternion:", msg.imu_state.quaternion)
        # print("RPY:", msg.imu_state.rpy)
        # print("Est. joint torque:", motor_torq)
            

input("Press enter to start")

if __name__ == '__main__':
    h1_ik = Robot_IK()

    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    # sub = ChannelSubscriber("rt/lowstate", LowState_)
    # sub.Init(LowStateMessageHandler, 10)

    state = State()
    state.__init__()

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

        # print("Joint state q:", state.q)
        tau = h1_ik.ik_func(state.q)
        # print("Grav. torque:", tau)

        motor_cmd = h1_ik.joint_torq(tau)
        # motor_cmd = h1_ik.joint_torq(tau_ref)
        # print("Grav. torque:", motor_cmd)


        # Total time for standing up or standing down is about 1.2s
        for i in range(kNumMotors):
            cmd.motor_cmd[i].q = stand_down_joint_pos[i]
            cmd.motor_cmd[i].kp = 50.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.0
            cmd.motor_cmd[i].tau = motor_cmd[i]

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
