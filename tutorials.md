---
layout: page
title: "Tutorials"
permalink: "/tutorials"
---
<script
  src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"
  type="text/javascript">
</script>

## <small>Linear Quadratic Regulator (LQR) control of a self balancing robot in ROS/Gazebo</small>

**Theory:**

The objective of discrete-time infinite horizon optimal control problem for linear time invariant (LTI) system is to obtain control inputs $u_0,u_1,\cdots$ that solve the following optimization problem
$$
\begin{aligned}
\min_{u_0,u_1,\cdots} &\sum_{j=0}^{\infty} (x_j^{\top}Qx_j+u_j^{\top}Ru_j) \\
 &x_{k+1} = Ax_k+Bu_k 
\end{aligned}
$$
where $Q\succeq 0$, $R\succ0$, $A$ and $B$ are matrices of appropriate dimension, and the pair $(A,B)$ is controllable. It is a regulator problem, where the aim is to bring the state to origin from an intial condition; extension to tracking problem can be made suitably. To solve, dynamic programming approach [1] is considered in the following. Consider the cost function defined as
$$
\begin{align*}
V(x_k)&\triangleq \sum_{j=k}^{\infty} (x_j^{\top}Qx_j+u_j^{\top}Ru_j)\\
V^*(x_k)& \triangleq  \min_{u_k,u_{k+1},\cdots} \sum_{j=k}^{\infty} (x_j^{\top}Qx_j+u_j^{\top}Ru_j)\\
& = \min_{u_k}\left(x_k^{\top}Qx_k+u_k^{\top}Ru_k+V^*(x_{k+1})\right). \tag{1}
\end{align*}
$$
Equation (1) can be rewritten as
$$
\begin{align*}
V^*(x_k) = \min_{u_k}\left(x_k^{\top}Qx_k+u_k^{\top}Ru_k+V^*(Ax_k+Bu_k)\right) \tag{2}
\end{align*}
$$
which is also called Hamilton-Jacobi equation. It can be shown that the optimal cost $V^*(x_k)$ is quadratic in $x_k$, i.e; $V^*(x_k) = x_k^{\top}Px_k$ for some $P\succ 0$ [2]. Hence (2) can be expressed as
$$
x_k^{\top}Px_k = \min_{u_k}\left(x_k^{\top}Qx_k+u_k^{\top}Ru_k+(Ax_k+Bu_k)^{\top}P(Ax_k+Bu_k)\right) \tag{3}
$$
and for optimal control $u_k^*$, (3) can be written as
$$
x_k^{\top}Px_k = x_k^{\top}Qx_k+(u_k^*)^{\top}R(u_k^*)+(Ax_k+Bu_k^*)^{\top}P(Ax_k+Bu_k^*) \tag{4}
$$
The optimal control that minimized right hand side of (3) can be easily obtained as
$$
u_k^*=-(R+B^{\top}PB)^{-1}B^{\top}PAx_k \tag{5} 
$$
and subtituting $u_k^*$ in (4) would lead to Algebraic Riccati Equation (ARE):
$$
P =Q +A^{\top}PA-A^{\top}PB(R+B^{\top}PB)^{-1}B^{\top}PA. \tag{6} 
$$
So the LQR consists of two steps: first solve ARE (6) to obtain $P$ and then use (5) when the algorithm is run in real time with state value $x_k$.

[1] Bertsekas, D., 2012. Dynamic programming and optimal control: Volume I (Vol. 4). Athena scientific.

[2] Lewis, F.L., Vrabie, D. and Syrmos, V.L., 2012. Optimal control. John Wiley & Sons.


<hr style="margin-top: -1em; margin-bottom: 1em;">

**Code repository:** [https://github.com/shaikshavali-chitraganti/LQR_SBR_Gazebo.git](https://github.com/shaikshavali-chitraganti/LQR_SBR_Gazebo.git) 



<iframe width="560" height="315" src="https://youtu.be/JwZwCb9DGGU" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
