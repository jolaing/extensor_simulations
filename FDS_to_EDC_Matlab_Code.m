%Joanna Laing (FCR_to_EDC_transfer)
%Predictive Simulation for FCR to EDC tendon transfer


%% Part 0: Load the Moco libraries and pre-configured Models.
import org.opensim.modeling.*;
% These models are provided for you (i.e., they are not part of Moco).
torqueDrivenModel = getTorqueDrivenModel();
muscleDrivenModel = getMuscleDrivenModel();

%% Part 1: Torque-driven Predictive Problem
% Part 1a: Create a new MocoStudy.
study = MocoStudy();

% Part 1b: Initialize the problem and set the model.
problem = study.updProblem();
problem.setModel(torqueDrivenModel);

% Part 1c: Set bounds on the problem.
%
% problem.setTimeBounds(initial_bounds, final_bounds)
% problem.setStateInfo(path, trajectory_bounds, inital_bounds, final_bounds)
%
% All *_bounds arguments can be set to a range, [lower upper], or to a
% single value (equal lower and upper bounds). Empty brackets, [], indicate
% using default bounds (if they exist). You may set multiple state infos at
% once using setStateInfoPattern():
%
% problem.setStateInfoPattern(pattern, trajectory_bounds, inital_bounds, ...
%       final_bounds)
%
% This function supports regular expressions in the 'pattern' argument;
% use '.*' to match any substring of the state/control path
% For example, the following will set all coordinate value state infos:
%
% problem.setStateInfoPattern('/path/to/states/.*/value', ...)

% Time bounds
problem.setTimeBounds(0, 3);

% Position bounds: the model should start with the hand in a 
% relaxed position anf finish with the fingers flat
problem.setStateInfo('/jointset/_2MCP/2mcp_flexion/value', ...
    [-0.785398, 1.57], 1.0472, 0);
problem.setStateInfo('/jointset/_2prox-midph_b/2pm_flexion/value', ...
    [0, 1.74533], 0.9948, 0);
problem.setStateInfo('/jointset/_2mid-distph/2md_flexion/value', ...
    [0, 1.39626], 0.6109, 0);
problem.setStateInfo('/jointset/_3MCP/3mcp_flexion/value', ...
    [-0.785398, 1.57], 1.0472, 0);
problem.setStateInfo('/jointset/_3prox-midph_b/3pm_flexion/value', ...
    [0, 1.74533], 0.9948, 0);
problem.setStateInfo('/jointset/_3mid-distph/3md_flexion/value', ...
    [0, 1.39626], 0.6109, 0);
problem.setStateInfo('/jointset/_4MCP/4mcp_flexion/value', ...
    [-0.785398, 1.57], 1.0472, 0);
problem.setStateInfo('/jointset/_4prox-midph_b/4pm_flexion/value', ...
    [0, 1.74533], 0.9948, 0);
problem.setStateInfo('/jointset/_4mid-distph/4md_flexion/value', ...
    [0, 1.39626], 0.6109, 0);
problem.setStateInfo('/jointset/_5MCP/5mcp_flexion/value', ...
    [-0.785398, 1.57], 1.0472, 0);
problem.setStateInfo('/jointset/_5prox-midph_b/5pm_flexion/value', ...
    [0, 1.74533], 0.9948, 0);
problem.setStateInfo('/jointset/_5mid-distph/5md_flexion/value', ...
    [0, 1.39626], 0.6109, 0);

% Velocity bounds: all model coordinates should start and end at rest.
problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0);

% Part 1d: Add a MocoControlGoal to the problem.
problem.addGoal(MocoControlGoal('myeffort'));

% Part 1e: Configure the solver.
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(25);
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);

if ~exist('predictSolution.sto', 'file')
% Part 1f: Solve! Write the solution to file, and visualize.
predictSolution = study.solve();
predictSolution.write('predictSolution.sto');
study.visualize(predictSolution);
end

%% Model Creation and Plotting Convenience Functions 

function addCoordinateActuator(model, coordName, optForce)

import org.opensim.modeling.*;

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();
actu.setName(['tau_' coordName]);
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optForce);
actu.setMinControl(-1);
actu.setMaxControl(1);

model.addComponent(actu);

end

function [model] = getTorqueDrivenModel()

import org.opensim.modeling.*;

% Load the base model.
model = Model('C:\Users\joann\OneDrive - University of Aberdeen\Models\FINGER EXTENSION MODELLING\FDS to EDC\Moco\post_surgery_fds_to_edc_2.osim');

% Remove the muscles in the model.
model.updForceSet().clearAndDestroy();
model.initSystem();

% Add CoordinateActuators to the model degrees-of-freedom.
addCoordinateActuator(model, '2mcp_flexion', 10);
addCoordinateActuator(model, '2pm_flexion', 5);
addCoordinateActuator(model, '2md_flexion', 3);
addCoordinateActuator(model, '3mcp_flexion', 10);
addCoordinateActuator(model, '3pm_flexion', 5);
addCoordinateActuator(model, '3md_flexion', 3);
addCoordinateActuator(model, '4mcp_flexion', 10);
addCoordinateActuator(model, '4pm_flexion', 5);
addCoordinateActuator(model, '4md_flexion', 3);
addCoordinateActuator(model, '5mcp_flexion', 10);
addCoordinateActuator(model, '5pm_flexion', 5);
addCoordinateActuator(model, '5md_flexion', 3);

end

function [model] = getMuscleDrivenModel()

import org.opensim.modeling.*;

% Load the base model.
model = Model('C:\Users\joann\OneDrive - University of Aberdeen\Models\FINGER EXTENSION MODELLING\FDS to EDC\Moco\post_surgery_fds_to_edc_2.osim');
model.finalizeConnections();

% Replace the muscles in the model with muscles from DeGroote, Fregly,
% et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem
% Formulations for Solving the Muscle Redundancy Problem". These muscles
% have the same properties as the original muscles but their characteristic
% curves are optimized for direct collocation (i.e. no discontinuities,
% twice differentiable, etc).
DeGrooteFregly2016Muscle().replaceMuscles(model);

% Make problems easier to solve by strengthening the model and widening the
% active force-length curve.
for m = 0:model.getMuscles().getSize()-1
    musc = model.updMuscles().get(m);
    musc.setMinControl(0);
    musc.set_ignore_activation_dynamics(false);
    musc.set_ignore_tendon_compliance(false);
    musc.set_max_isometric_force(2 * musc.get_max_isometric_force());
    dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
    dgf.set_active_force_width_scale(1.5);
    dgf.set_tendon_compliance_dynamics_mode('implicit');
    if strcmp(char(musc.getName()), 'soleus_r')
        % Soleus has a very long tendon, so modeling its tendon as rigid
        % causes the fiber to be unrealistically long and generate
        % excessive passive fiber force.
        dgf.set_ignore_passive_fiber_force(true);
    end
end

end