= Hinge Constraint
== Velocity

Hinge constraint is defined by: \
- $b_1$, $b_2$: ActiveBody \
- $a_1$, $a_2$: Hinge axes (local) \
- $n_1$, $n_2$: Normal axes (local) \
- $a_min$, $a_max$: Max angles \
- $p_1$, $p_2$: Pivot points (local) \

$


$

- $p_1$, $p_2$ make simple point constraint, $r_1$, $r_2$, $r_j = b_j_p + R_j * p_j$.
- Rotation around the hinge axis is obtained by first finding $a_(1 r)$, $a_(2 r)$ hinge axes where $a_(j r) = R_j * a_j$, where $R_j$ is rotation matrix of $b_j$. Then find 2 vectors $b$, $c$ s.t. $b perp c perp a_(2 r)$. Thus this part of the hinge constraint is:
$

mat(
  a_(1 r) . b;
  a_(1 r) . c
) = 0

$

- To constrain rotation around the hinge axis, we first find $n_(1 r)$, $n_(2 r)$ where $n_(j r) = R_j * n_j$. Then $theta$ is signed angle between $hat(u) = n_(1 r)$ and $hat(v) = ("rej"_(a_(1 r))n_(2 r)) / (||"rej"_(a_(1 r))n_(2 r)||)$. $theta$ can be found as $"atan2"((hat(u) times hat(v)) dot a_(1 r), hat(u) dot hat(v))$.

Assembling these constraints and differentiating w.r.t. time yields the Jacobian:

#grid(
  columns: (1fr, 1fr),
  [
    $
      J = mat(
        -n, -r_1 times n, n, r_2 times n;
        0, -b times a_(1 r), 0, b times a_(1 r);
        0, -c times a_(1 r), 0, c times a_(1 r);
        0, -a_(1 r), 0, a_(1, r)
      ) \
      
      lambda = (J M^(-1) J^T )^(-1) (- J V - b) \
      K = J M^(-1) J^T \

      E_12 = J_12 I^(-1)_1 \
      E_14 = J_14 I^(-1)_2 \
      D_1 = E_12 - E_14 \
      \
      E_22 = J_22 I^(-1)_1 \
      E_24 = -J_22 I^(-1)_2 \
      D_2 = E_22 - E_24 \
      \
      E_32 = J_32 I^(-1)_1 \
      E_34 = -J_32 I^(-1)_2 \
      D_3 = E_32 - E_34 \
      \
      E_42 = J_42 I^(-1)_1 \
      E_44 = -J_42 I^(-1)_2 \
      D_4 = E_42 - E_44 \
    $
  ],[
    $
    
    B = beta / (Delta t) vec(
      ||r_2 - r_1||,
      a_(1 r) dot b,
      a_(1 r) dot c,
      theta - a
  ) \
    
    M = mat(
      E_3 / m_1, 0, 0, 0;
      0, I^(-1)_1, 0, 0;
      0, 0, E_3 / m_2, 0;
      0, 0, 0, I^(-1)_2
    ) \
    
    \
    
    K = mat(
      (1 / m_1 + E_12 J_12^T + 1 / m_2 + E_14 J_14^T),
      D_1 J_22^T,
      D_1 J_32^T,
      D_1 J_42^T;
      .,
      D_2 J_22^T,
      D_2 J_32^T,
      D_2 J_42^T;
      .,
      .,
      D_3 J_32^T,
      D_3 J_42^T;
      .,
      .,
      .,
      D_4 J_42^T;
    ) \

    Delta V = M^(-1) J^T lambda
    
    $
  ]
)

$K^(-1)$ is simply $"adj"(K)/det(K)$, which is fine to calculate for small $RR^(4 times 4)$ matrix. Since this gets reused over multiple iterations, we'll calculate and store it explicitly while $-J V + b$ varies.

For solving, will store the upper triangle of $K^(-1)$, $J_11$, $J_12$, $J_14$, $J_22$, $J_32$, $J_42$, and the $b$ bias vector.

== Position

The full constraint vector is:

$

C = vec(||r_2 - r_1||, a_(1 r) dot b, a_(1 r) dot c, theta - a) = 0

$

We also have that $C(x + Delta x) = C(x) + integral_(x)^(x + Delta x) (d C(k)) / (d x) d k$. The first order Taylor approximation of this yields 
 $
  C(X) = E \
  (d C(X)) / (d X) = J \
  C(X + Delta X) = E + J Delta X = 0 \
  Delta X = M^(-1) J^T lambda \
  E + J M^(-1) J^T lambda = 0 \
  lambda = (J M^(-1) J^T)^(-1) (-E) \
$

By the chain rule, $(d C) / (d t) = (d C) / (d X) (d X) / (d t)$, we can match terms to see that $(d C) / (d X) = J$.

The problem with limited constraints is that a position solve step might not need to fix a particular constraint. This means that the derivative of the constraint is 0 at that point, leaving a row of $J$ as all 0, meaning the determinant is 0.

== Rework

The previous method of dealing with the position constraint via just the normal doesn't work for small normals and is inaccurate. The updated constraint is:
$
  C(X) = x_2 + r_2 - x_1 - r_1 = 0\
$
$
  (d C (X)) / (d t) &= v_2 + omega_2 times r_2 - v_1 - omega_1 times r_1 \
  &= v_2 - [r_2]_x w_2 - v_1 - [r_1]_x w_1 \
  &= mat(-E_3, [r_1]_x, E_3, -[r_2]_x) vec(v_1, omega_1, v_2, omega_2)
$

Where:
$
a times b = [a]_x b = mat(0, -a_3, a_2; a_3, 0, -a_1; -a_2, a_1, 0) vec(b_1, b_2, b_3) = [b]_x^T a
$

Thus the full Jacobian is now:
$
      J = mat(
        mat(-1, 0, 0), mat(0, -r_(1 z), r_(1 y)), mat(1, 0, 0), mat(0, r_(2 z), -r_(2 y));
        mat(0, -1, 0), mat(r_(1 z), 0, -r_(1 x)), mat(0, 1, 0), mat(-r_(2 z), 0, r_(2 x));
        mat(0, 0, -1), mat(-r_(1 y), r_(1 x), 0), mat(0, 0, 1), mat(r_(2 y), -r_(2 x), 0);
        0, -b times a_(1 r), 0, b times a_(1 r);
        0, -c times a_(1 r), 0, c times a_(1 r);
        0, -a_(1 r), 0, a_(1, r)
      ) \
$

#text(
  size: 10pt
)[
  $
    K = J M^(-1) J^T = mat(
      (1 /m_1 + E_12 J_12^T + 1/m_2 + E_14 J_14^T), E_12 J_22^T + E_14 J_24^T, E_12 J_32^T + E_14 J_34^T, D_1 J_42^T, D_1 J_52^T, D_1 J_62^T;
       , (1 /m_1 + E_22 J_22^T + 1/m_2 + E_24 J_24^T), E_22 J_32^T + E_24 J_34^T, D_2 J_42^T, D_2 J_52^T, D_2 J_62^T;
        , , (1 /m_1 + E_32 J_32^T + 1/m_2 + E_34 J_34^T), D_3 J_42^T, D_3 J_52^T, D_3 J_62^T;
         , , , D_4 J_42^T, D_4 J_52^T, D_4 J_62^T;
          , , , , D_5 J_52^T, D_5 J_62^T;
          ("symmetric"), , , , , D_6 J_62^T
    )
  $
]
$
D_1 = E_12 - E_14 \
D_2 = E_22 - E_24 \
D_3 = E_32 - E_34 \
D_4 = E_42 - E_44 \
D_5 = E_52 - E_54 \
D_6 = E_62 - E_64 \
$

We need to evaluate:
$
  K^(-1) (-J V - b) = lambda \
  -J V - b = K lambda
$

This is in the form $A x = b$ where $A = K, k = lambda, b = - J V - b$, so we'll solve it via Cholensky decomposition.
