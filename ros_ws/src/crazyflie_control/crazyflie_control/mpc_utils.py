import numpy as np
import casadi as cas
import ray
from .control_packagecf import *


def setup_solver(solver_config):
    n, du = 6, 3
    Ts = solver_config['Ts']
    Npred = solver_config['Npred']
    Q = solver_config['Q']
    R = solver_config['R']

    A = np.block([[np.zeros((3, 3)), np.eye(3)], [np.zeros((3, 6))]])
    B = np.block([[np.zeros((3, 3))], [np.eye(3)]])
    A_d = np.eye(6) + Ts * A
    # B_d = B * Ts
    B_d = np.block([[Ts * Ts / 2 * np.eye(3)], [Ts * np.eye(3)]])

    data_Upos = np.load('src/crazyflie_control/crazyflie_control/Upos.npy', allow_pickle=True).tolist()
    Vc = {}
    Vc['A_vc'] = np.round(data_Upos['A'][0, 0], 5)  # round up
    Vc['b_vc'] = np.round(data_Upos['b'][0, 0], 5)

    solver = cas.Opti()

    x = solver.variable(n, Npred + 1)
    v = solver.variable(du, Npred)
    xinit = solver.parameter(n, 1)
    vinit = solver.parameter(du, 1)
    xref_param = solver.parameter(n, Npred)
    vref_param = solver.parameter(du, Npred)
    psi_ref_param = solver.parameter(1, 1)

    # Set constraints
    solver.subject_to(x[:, 0] == xinit)
    for k in range(Npred):
        solver.subject_to(x[:, k+1] == A_d @ x[:, k] + B_d @ v[:, k])
        solver.subject_to(cas.mtimes(Vc['A_vc'], v[:, k]) <= Vc['b_vc'])

    # Set objective
    objective = 0
    for k in range(Npred):
        state_error = x[:, k] - xref_param[:, k]
        # control_effort = v[:, k] - (v[:, k-1] if k > 0 else vinit)
        control_effort = v[:, k] - vref_param[:, k]
        objective += cas.mtimes([state_error.T, Q, state_error]) + cas.mtimes([control_effort.T, R, control_effort])

    solver.minimize(objective)
    opts = {"ipopt.print_level": 0, "print_time": False, "ipopt.sb": "yes"}
    solver.solver('ipopt', opts)

    solver.set_value(vinit, np.zeros(3))

    return {
        "solver": solver,
        "x": x, "v": v, "xinit": xinit, "vinit": vinit,
        "xref_param": xref_param, "psi_ref_param": psi_ref_param, "vref_param": vref_param,
        "Npred": Npred, "Vc": Vc, "A_d": A_d, "B_d": B_d, "Q": Q, "R": R
    }


def compute_control_real(solver_data, state_xi, pos_ref, v_ref, i, id=0):

    if i + solver_data["Npred"] <= len(pos_ref):
        # desired_pos = pos_ref[i:i+solver_data["Npred"], 0:3] + np.tile(self.formation_offsets[id].reshape(1, -1), (solver_data["Npred"], 1))
        desired_pos = pos_ref[i:i+solver_data["Npred"], 0:3]
        ref = np.vstack([desired_pos.T, pos_ref[i:i+solver_data["Npred"], 3:6].T])
        virt_ref = v_ref[i:i+solver_data["Npred"], 0:3].T
    else:
        # If there arent sufficient samples left in the trajectory, stack the last point of the ref
        remaining = len(pos_ref) - i
        desired_pos = np.vstack([pos_ref[i:, 0:3], np.tile(pos_ref[-1, 0:3], (solver_data["Npred"] - remaining, 1))])
        desired_vel = np.vstack([pos_ref[i:, 3:6], np.tile(pos_ref[-1, 3:6], (solver_data["Npred"] - remaining, 1))])
        virt_ref = np.vstack([v_ref[i:, 0:3], np.tile(v_ref[-1, 0:3], (solver_data["Npred"] - remaining, 1))])
        ref = np.vstack([desired_pos.T, desired_vel.T])
        virt_ref = virt_ref.T

    solver_data["solver"].set_value(solver_data["xinit"], state_xi[:, 0:6].T)
    solver_data["solver"].set_value(solver_data["xref_param"], ref)
    solver_data["solver"].set_value(solver_data["vref_param"], virt_ref)
    solver_data["solver"].set_value(solver_data["psi_ref_param"], 0)

    sol = solver_data["solver"].solve()
    vopt = sol.value(solver_data["v"])
    return [float(vopt[0, 0]), float(vopt[1, 0]), float(vopt[2, 0])]


@ray.remote
class MPCWorker:
    def __init__(self, agent_id, drone, uri, solver_config):
        self.id = agent_id
        self.drone = drone
        self.uri = uri
        self.solver_data = setup_solver(solver_config)
        self.Na = solver_config['Na']
        self.Npred = self.solver_data['Npred']

    def solve(self, states, pos_ref, v_ref, i):

        pos_ref = pos_ref[self.uri]
        v_ref = v_ref[self.uri]

        state_xi = states[self.drone][:, 0:6].T
        _, _, yaw_tmp = quaternion_to_euler(states[self.drone][:, 6:10][0])

        solver = self.solver_data['solver']

        if i + self.Npred <= len(pos_ref):
            desired_pos = pos_ref[i:i+self.Npred, 0:3]
            ref = np.vstack([desired_pos.T, pos_ref[i:i+self.Npred, 3:6].T])
            virt_ref = v_ref[i:i+self.solver_data["Npred"], 0:3].T
        else:
            remaining = len(pos_ref) - i
            desired_pos = np.vstack([pos_ref[i:, 0:3], np.tile(pos_ref[-1, 0:3], (self.Npred - remaining, 1))])  # stack the last position
            desired_vel = np.vstack([pos_ref[i:, 3:6], np.tile(pos_ref[-1, 3:6], (self.Npred - remaining, 1))])
            virt_ref = np.vstack([v_ref[i:, 0:3], np.tile(v_ref[-1, 0:3], (self.Npred - remaining, 1))])
            ref = np.vstack([desired_pos.T, desired_vel.T])
            virt_ref = virt_ref.T

        solver.set_value(self.solver_data['xinit'], state_xi[:, 0:6].T)
        solver.set_value(self.solver_data['xref_param'], ref)
        solver.set_value(self.solver_data['vref_param'], virt_ref)
        solver.set_value(self.solver_data['psi_ref_param'], 0)

        sol = solver.solve()
        vopt = sol.value(self.solver_data['v'])
        vopt = [float(vopt[0, 0]), float(vopt[1, 0]), float(vopt[2, 0])]
        return self.id, vopt, yaw_tmp


@ray.remote
class MPCWorkerCFT:
    def __init__(self, agent_id, drone, uri, solver_config):
        self.id = agent_id
        self.drone = drone
        self.uri = uri
        self.solver_data = setup_solver(solver_config)
        self.Na = solver_config['Na']
        self.Npred = self.solver_data['Npred']

    def solve(self, states, pos_ref, v_ref, i, drone_index_in_cluster, cluster_size):

        state_xi = states[self.drone][:, 0:6].T
        _, _, yaw_tmp = quaternion_to_euler(states[self.drone][:, 6:10][0])

        offsets = generate_formation_offsets(cluster_size, d=1)
        offset = offsets[drone_index_in_cluster]

        solver = self.solver_data['solver']

        if i + self.Npred <= len(pos_ref):
            desired_pos = pos_ref[i:i+self.Npred, 0:3] + offset
            ref = np.vstack([desired_pos.T, pos_ref[i:i+self.Npred, 3:6].T])
            virt_ref = v_ref[i:i+self.solver_data["Npred"], 0:3].T
        else:
            remaining = len(pos_ref) - i  # stack the last position for as many samples as is necessary
            desired_pos = np.vstack([pos_ref[i:, 0:3], np.tile(pos_ref[-1, 0:3], (self.Npred - remaining, 1))]) + offset
            desired_vel = np.vstack([pos_ref[i:, 3:6], np.tile(pos_ref[-1, 3:6], (self.Npred - remaining, 1))])
            virt_ref = np.vstack([v_ref[i:, 0:3], np.tile(v_ref[-1, 0:3], (self.Npred - remaining, 1))])
            ref = np.vstack([desired_pos.T, desired_vel.T])
            virt_ref = virt_ref.T

        solver.set_value(self.solver_data['xinit'], state_xi[:, 0:6].T)
        solver.set_value(self.solver_data['xref_param'], ref)
        solver.set_value(self.solver_data['vref_param'], virt_ref)
        solver.set_value(self.solver_data['psi_ref_param'], 0)

        sol = solver.solve()
        vopt = sol.value(self.solver_data['v'])
        vopt = [float(vopt[0, 0]), float(vopt[1, 0]), float(vopt[2, 0])]
        return self.id, vopt, yaw_tmp
