import numpy as np
from partition.Propagator import Propagator
import torch
import pypoman
from closed_loop.reach_sdp import getE_in, getE_mid, getE_out, getInputConstraints, getOutputConstraints, getNNConstraints
import cvxpy as cp
from tqdm import tqdm

class ClosedLoopPropagator(Propagator):
    def __init__(self, input_shape=None, At=None, bt=None, ct=None):
        Propagator.__init__(self, input_shape=input_shape)
        self.At = At
        self.bt = bt
        self.ct = ct

    def get_one_step_reachable_set(self, A_inputs, b_inputs, A_out, u_limits=[-5., 5.]):
        raise NotImplementedError

    def get_reachable_set(self, A_inputs, b_inputs, A_out, t_max, u_limits=None):
        all_bs = []
        bs, _ = self.get_one_step_reachable_set(A_inputs, b_inputs, A_out, u_limits=u_limits)
        all_bs.append(bs)
        for i in range(1, t_max):
            bs, _ = self.get_one_step_reachable_set(A_out, bs, A_out, u_limits=u_limits)
            all_bs.append(bs)
        return all_bs, {}

class ClosedLoopSDPPropagator(ClosedLoopPropagator):
    def __init__(self, input_shape=None, At=None, bt=None, ct=None):
        ClosedLoopPropagator.__init__(self, input_shape=input_shape, At=At, bt=bt, ct=ct)

    def torch2network(self, torch_model):
        return torch_model

    def get_one_step_reachable_set(self, A_inputs, b_inputs, A_out, u_limits=[-5., 5.]):
        # Count number of units in each layer, except last layer
        # For keras:
        # num_neurons = np.sum([layer.get_config()['units'] for layer in self.network.layers][:-1])
        # For torch:
        num_neurons = np.sum([layer.out_features for layer in self.network if isinstance(layer, torch.nn.Linear)][:-1])

        # Number of vertices in input polyhedron
        m = A_inputs.shape[0]
        num_states = self.At.shape[0]
        num_inputs = self.bt.shape[1]

        u_min, u_max = u_limits

        # Get change of basis matrices
        E_in = getE_in(num_states, num_neurons, num_inputs)
        E_mid = getE_mid(num_states, num_neurons, num_inputs, self.network, u_min, u_max)
        E_out = getE_out(num_states, num_neurons, num_inputs, self.At, self.bt, self.ct)

        # Get P,Q,S and constraint lists
        P, input_set_constrs = getInputConstraints(num_states, m, A_inputs, b_inputs)
        Q, nn_constrs = getNNConstraints(num_neurons, num_inputs)

        # M_in describes the input set in NN coords
        M_in = cp.quad_form(E_in, P)
        M_mid = cp.quad_form(E_mid, Q)

        num_facets = A_inputs.shape[0]
        bs = np.zeros((num_facets))
        for i in tqdm(range(num_facets)):
            S_i, reachable_set_constrs, b_i = getOutputConstraints(num_states, A_inputs[i,:])
            M_out = cp.quad_form(E_out, S_i)

            constraints = input_set_constrs + nn_constrs + reachable_set_constrs

            constraints.append(M_in + M_mid + M_out << 0)

            objective = cp.Minimize(b_i)
            prob = cp.Problem(objective,
                              constraints)
            prob.solve()
            # print("status:", prob.status)
            bs[i] = b_i.value

        # print("status:", prob.status)
        # print("optimal value", prob.value)
        # print("b_{i}: {val}".format(i=i, val=b_i.value))
        # print("S_{i}: {val}".format(i=i, val=S_i.value))

        return bs, {}

class ClosedLoopCROWNIBPCodebasePropagator(ClosedLoopPropagator):
    def __init__(self, input_shape=None, At=None, bt=None, ct=None):
        ClosedLoopPropagator.__init__(self, input_shape=input_shape, At=At, bt=bt, ct=ct)

    def torch2network(self, torch_model):
        from closed_loop.nn_bounds import BoundClosedLoopController
        torch_model_cl = BoundClosedLoopController.convert(torch_model, self.params,
            A_dyn=torch.Tensor([self.At]), b_dyn=torch.Tensor([self.bt]), c_dyn=[self.ct])
        return torch_model_cl

    def forward_pass(self, input_data):
        return self.network(torch.Tensor(input_data), method_opt=None).data.numpy()

    def get_one_step_reachable_set(self, A_inputs, b_inputs, A_out, u_limits=None):
        # Get bounds on each state from A_inputs, b_inputs
        num_states = self.At.shape[0]
        vertices = pypoman.compute_polygon_hull(A_inputs, b_inputs)
        x_max = []
        x_min = []
        for state in range(num_states):
            x_max.append(np.max([v[state] for v in vertices]))
            x_min.append(np.min([v[state] for v in vertices]))
        
        num_facets = A_out.shape[0]
        bs = np.zeros((num_facets))
        for i in range(num_facets):
            xt1_max, _, xt1_min, _ = self.network.full_backward_range(norm=np.inf,
                                        x_U=torch.Tensor([x_max]),
                                        x_L=torch.Tensor([x_min]),
                                        upper=True, lower=True, C=torch.Tensor([[[1]]]),
                                        A_out=torch.Tensor([A_out[i,:]]),
                                        A_in=A_inputs, b_in=b_inputs,
                                        u_limits=u_limits)
            bs[i] = xt1_max
        return bs, {}

class ClosedLoopIBPPropagator(ClosedLoopCROWNIBPCodebasePropagator):
    def __init__(self, input_shape=None, At=None, bt=None, ct=None):
        ClosedLoopCROWNIBPCodebasePropagator.__init__(self, input_shape=input_shape, At=At, bt=bt, ct=ct)
        self.method_opt = "interval_range"
        self.params = {}

class ClosedLoopCROWNPropagator(ClosedLoopCROWNIBPCodebasePropagator):
    def __init__(self, input_shape=None, At=None, bt=None, ct=None):
        ClosedLoopCROWNIBPCodebasePropagator.__init__(self, input_shape=input_shape, At=At, bt=bt, ct=ct)
        self.method_opt = "full_backward_range"
        self.params = {"same-slope": False}

class ClosedLoopFastLinPropagator(ClosedLoopCROWNIBPCodebasePropagator):
    def __init__(self, input_shape=None, At=None, bt=None, ct=None):
        ClosedLoopCROWNIBPCodebasePropagator.__init__(self, input_shape=input_shape, At=At, bt=bt, ct=ct)
        self.method_opt = "full_backward_range"
        self.params = {"same-slope": True}

