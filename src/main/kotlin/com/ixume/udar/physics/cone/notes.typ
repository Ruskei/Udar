== Cone Constraint
Defined by:
+ parent body $b_p$
+ child body $b_c$
+ child swing axis $Z$ and reference axis $X_c$
+ parent $X$, $Y$ axes
+ min, max x anx y rotation $theta_"max"$, $phi_"max"$
+ min, max twist $psi_"min"$, $psi_"max"$
+ relative pivot points $r_p$, $r_c$

Simple position constraint handles translation.

Rotation is constrained by the parent's X and Y axes; the rotation of Z with respect to X and Y is $theta$, $phi$. We find $theta, phi$ relative to parent Z axis which is simply $X times Y$. Thus $theta$ can be found by taking the dot product between the child Z and parent Z after rejecting the relevant axis. $alpha_"max" = sin(theta_"max"), beta_ = sin(phi_"max")$. If $sin(theta)^2 / alpha^2 + sin(phi)^2 / beta^2 > 1$ then the constraint is violated. We know the slope of the line in XY space as $k = sin(phi)/sin(theta)$, and we can project a point onto the ellipse by solving 
$
lambda^2 / alpha^2 + (lambda k)^2 / beta^2 = 1 \
lambda^2 beta^2 + lambda^2 k^2 alpha^2 = alpha^2 beta^2 \
lambda^2 = (alpha^2 beta^2) / (beta^2 + k^2 alpha^2) \
lambda = (alpha beta) / sqrt(k alpha^2 + beta^2)
$

We only take the positive solution to not flip signs. Our clamped point is then $p = (lambda, lambda k)$. $theta_"clamped" = arcsin(lambda), phi_"clamped" = arcsin(lambda k)$, which are used for errors and biases.

To determine twist we simply find the signed angle between $X_c$ and $X$ on the child $X, Y$ plane. $X_2 = "rej"_Z X$, $cos(psi) = X_2 dot X_c, sin(psi) = (X_2 times X_c) dot Z$, thus $psi = "atan2"(sin(psi), cos(psi))$.

The constraint is in one of 4 states: no angle limits, z-limited, xy-limited (technically could be just one but this is a rare case), or xyz-limited. This means the Jacobian can be 3, 4, 5, or 6 dimensional. The full 6D constraint be:

$
  C = mat(
    x_1 + r_1 - x_2 - r_2 = 0;
    theta <= theta_max;
    phi <= phi_max;
    psi_"min" <= psi <= psi_"max"
  )
$

$
J = mat(
  -E_3, [r_1]_times, E_3, -[r_2]_times;
  0, -cal(X), 0, cal(X);
  0, -cal(Y), 0, cal(Y);
  0, -cal(Z), 0, cal(Z)
)
$

Intuitively, the rows correspond to: stopping rotation in $theta$, $phi$, $psi$. Since $psi$ is measured by finding the angle between $X_c$ and $X$ on the child's $X Y$ plane (formed by rejecting $Z$), the rotation to fix it must be around the $Z$ axis.
