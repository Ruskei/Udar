= Hinge Constraint

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
      (theta - a)
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
    )
    
    $
  ]
)

$K^(-1)$ is simply $"adj"(K)/det(K)$, which is fine to calculate for small $RR^(4 times 4)$ matrix. Since this gets reused over multiple iterations, we'll calculate and store it explicitly while $-J V + b$ varies.

For solving, will store the upper triangle of $K^(-1)$, $J_11$, $J_12$, $J_14$, $J_22$, $J_32$, $J_42$, and the $b$ bias vector.
