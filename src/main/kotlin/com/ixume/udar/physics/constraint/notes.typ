== Ax = b Solving

For a symmetric matrix $A$, its Cholensky Decomposition is $L L^T$ where L is a lower triangular matrix. This can be written as:
$
A =
mat(
  l_11, 0, 0, 0;
  l_12, l_22, 0, 0;
  l_13, l_23, l_33, 0;
  l_14, l_24, l_34, l_44
)
mat(
  l_11, l_12, l_13, l_14;
  0, l_22, l_23, l_24;
  0, 0, l_33, l_34;
  0, 0, 0, l_44;
)

=

mat(
  l_11^2, l_11 l_12, l_11 l_13, l_11 l_14;
   , l_12^2 + l_22^2, l_12 l_13 + l_22 l_23, l_12 l_14 + l_22 l_24;
  , , l_13^2 + l_23^2 + l_33^2, l_13 l_14 + l_23 l_24 + l_33 l_34;
  ("symmetric"), , , l_14^2 + l_24^2 + l_34^2 + l_44^2
) \

a_11 = l_11^2 \
a_12 = l_11 l_12 \
a_13 = l_11 l_13 \
a_22 = l_12^2 + l_22^2 \
a_23 = l_12 l_13 + l_22 l_23 \
a_33 = l_13^2 + l_23^2 + l_33^2
$

$
l_(1i) = a_(1i) / l_11
$

$
l_23 = (a_23 - l_12 l_13) / l_22 \
l_24 = (a_24 - l_12 l_14) / l_22 \
l_34 = (a_34 - l_13 l_14 - l_23 l_24) / l_33
$

$
l_(i j) = (a_(i j) - (l_(1 i) ... l_((i - 1) i) ) dot (l_(1 j) ... l_((i - 1) j))) /l_(i i), i != j \
l_(i i) = sqrt(a_(i i) - sum(l_(1 i)^2 ... l_((i - 1) i)^2))
$

Then use back and forward substitution to find $x$.
$
L L^T x = b \
L^T x = L^(-1) b\
L y = b \
y = L^(-1) b \
L^T x = y
$
