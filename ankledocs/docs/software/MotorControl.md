### Motor control theory 

The user-commanded position, velocity, torque, proportional gain, and derivative gain are restricted to a commanded operating range, referred to as the **software constraint set**, (\mathcal{S}).

A second, extended admissible range is defined as the **running constraint set**, (\mathcal{R}). This range represents the absolute limits within which the motor is permitted to operate based on its measured state. The two constraint sets satisfy

\(\mathcal{S}\subseteq\mathcal{R}\)

and, when the running constraint is strictly wider than the software constraint,


\(\mathcal{S}\subsetneq\mathcal{R}\)


Consequently, a user command must satisfy


\(\mathbf{u}_{\mathrm{cmd}}\in\mathcal{S}\),


whereas the measured motor state must remain within


\(\mathbf{x}_{\mathrm{motor}}\in\mathcal{R}.\)

The region (\mathcal{R}\setminus\mathcal{S}) provides a recovery margin that accommodates tracking error, system dynamics, and externally imposed motion while preventing the user from directly commanding the motor near its absolute operating limits.
