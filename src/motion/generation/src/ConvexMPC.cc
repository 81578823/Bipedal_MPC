#include "generation/ConvexMPC.h"
#include <pinocchio/Orientation.h>
#include <rcpputils/asserts.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

/**
 * @brief ConvexMPC构造函数 / ConvexMPC Constructor
 * 
 * 初始化凸优化模型预测控制器，加载配置参数并设置求解器
 * Initialize convex model predictive controller, load configuration parameters and setup solver
 */
ConvexMPC::ConvexMPC(Node::SharedPtr nodeHandle,
                     std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
                     std::shared_ptr<ReferenceBuffer> referenceBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      referenceBuffer_(referenceBuffer) {

  // 加载配置文件 / Load configuration file
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  base_name = config_["model"]["base_name"].as<std::string>();
  scalar_t freq_ = config_["generation"]["frequency"].as<scalar_t>();
  dt_ = 0.02;  // MPC时间步长 / MPC time step

  // 获取机器人总质量 / Get robot total mass
  total_mass_ = pinocchioInterface_ptr_->total_mass();
  // weight_.diagonal() << 10, 10, 20, 0.1, 0.1, 0.1, 1, 1, 1, 0.1, 0.1, 0.1;
  // 设置状态权重矩阵 / Set state weight matrix
  // [位置x,y,z, 速度vx,vy,vz, 姿态角θx,θy,θz, 角速度ωx,ωy,ωz]
  // [position x,y,z, velocity vx,vy,vz, orientation θx,θy,θz, angular velocity ωx,ωy,ωz]
  weight_.setZero(12, 12);
  weight_.diagonal() << 20, 20, 40, 0.2, 0.2, 0.1, 4, 4, 4, 2.0, 2.0, 3.0;
  // 增加yaw(0.1->1.0)和w_z(0.1->0.5)的权重以提高稳定性
  // Increase yaw and w_z weights to improve stability

  // 配置HPIPM求解器参数 / Configure HPIPM solver parameters
  solver_settings.mode = hpipm::HpipmMode::Speed;
  solver_settings.iter_max = 30;          // 最大迭代次数 / Maximum iterations
  solver_settings.alpha_min = 1e-8;       // 最小步长 / Minimum step size
  solver_settings.mu0 = 1e2;              // 初始障碍参数 / Initial barrier parameter
  solver_settings.tol_stat = 1e-04;       // 稳态容差 / Stationarity tolerance
  solver_settings.tol_eq = 1e-04;         // 等式约束容差 / Equality constraint tolerance
  solver_settings.tol_ineq = 1e-04;       // 不等式约束容差 / Inequality constraint tolerance
  solver_settings.tol_comp = 1e-04;       // 互补性容差 / Complementarity tolerance
  solver_settings.reg_prim = 1e-12;       // 原始正则化 / Primal regularization
  solver_settings.pred_corr = 1;          // 预测-校正模式 / Predictor-corrector mode
  solver_settings.ric_alg = 0;            // Riccati算法类型 / Riccati algorithm type
  solver_settings.split_step = 1;         // 分步求解 / Split step solving

  // 初始化速度命令 / Initialize velocity commands
  vel_cmd.setZero();
  yawd_ = 0.0;
  t0 = nodeHandle->now().seconds();
  RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"), "ConvexMPC: Construction done");
}

ConvexMPC::~ConvexMPC() {}

/**
 * @brief 设置速度命令 / Set velocity command
 * @param vd 期望线速度 / Desired linear velocity
 * @param yawd 期望偏航角速度 / Desired yaw rate
 * 
 * 设置机器人的期望运动速度，包含滤波和限制处理
 * Set robot desired motion velocity with filtering and limiting
 */
void ConvexMPC::setVelCmd(vector3_t vd, scalar_t yawd) {
  vel_cmd = vd;
  // 渐进式角速度限制，避免突变 / Progressive angular velocity limiting to avoid sudden changes
  scalar_t yawd_target = min(max(yawd, -0.3), 0.3);
  // yawd_ = yawd_target;
  yawd_ = 0.9 * yawd_ + 0.1 * yawd_target;  // 低通滤波 / Low-pass filtering
}

/**
 * @brief 设置高度命令 / Set height command
 * @param h 期望高度 / Desired height
 */
void ConvexMPC::setHeightCmd(scalar_t h) { h_des = h; }

/**
 * @brief 生成参考轨迹 / Generate reference trajectory
 * 
 * 根据当前状态和用户命令生成机器人的参考运动轨迹
 * Generate robot reference motion trajectory based on current state and user commands
 */
void ConvexMPC::generateTrajRef() {
  const scalar_t t_now = nodeHandle_->now().seconds();
  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);

  // 首次运行初始化 / First run initialization
  if (first_run) {
    first_run = false;
    pos_start = base_pose_m.translation();
    pos_start.z() = h_des;  // 设置期望高度 / Set desired height
    rpy_start = toEulerAngles(base_pose_m.rotation());
    rpy_start.head(2).setZero();  // 保持roll和pitch为零 / Keep roll and pitch zero
  } else {
    // 连续运行时的轨迹更新 / Trajectory update during continuous operation
    vector_t rpy_c = toEulerAngles(base_pose_m.rotation());
    if (computeEulerAngleErr(rpy_c, rpy_start).norm() < 0.3) {
      rpy_start += dt_ * vector3_t(0, 0, yawd_);  // 更新yaw角度 / Update yaw angle
    }
    rpy_start.head(2).setZero();  // 保持roll和pitch为零 / Keep roll and pitch zero

    // 计算世界坐标系下的速度命令 / Calculate velocity command in world frame
    vector3_t vw = base_pose_m.rotation() * vel_cmd;
    /* if ((base_pose_m.translation() - pos_start).norm() < 0.2) {
      pos_start += dt_ * vw;
      pos_start.z() = h_des;
    } */
    // 更新期望位置 / Update desired position
    pos_start.head(2) = base_pose_m.translation().head(2) + dt_ * vw.head(2);
    pos_start.z() = h_des;  // 保持期望高度 / Maintain desired height
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> rpy_t, pos_t;
  scalar_t horizon_time = referenceBuffer_->getModeSchedule()->duration();
  size_t N = horizon_time / 0.02;
  for (size_t k = 0; k < N; k++) {
    time.push_back(t_now + 0.02 * k);
    vector3_t rpy_k = rpy_start;
    rpy_k.z() += 0.02 * k * yawd_;
    rpy_t.emplace_back(rpy_k);

    vector3_t vel_des = toRotationMatrix(rpy_k) * vel_cmd;
    vector3_t pos_k = pos_start + 0.02 * k * vel_des;
    pos_k.z() = h_des;
    pos_t.emplace_back(pos_k);
  }

  auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_pos->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_pose_m.rotation() * vel_cmd,
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_pos->fit(time, pos_t);
  referenceBuffer_->setIntegratedBasePosTraj(cubicspline_pos);

  auto cubicspline_rpy = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_rpy->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector3_t(0, 0, yawd_),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_rpy->fit(time, rpy_t);
  referenceBuffer_->setIntegratedBaseRpyTraj(cubicspline_rpy);
}

/**
 * @brief 获取系统动力学模型 / Get system dynamics model
 * @param time_cur 当前时间 / Current time
 * @param k 预测步长索引 / Prediction horizon index
 * @param mode_schedule 步态调度器 / Mode scheduler
 * 
 * 构建线性化的双足机器人动力学模型，用于MPC预测
 * Build linearized bipedal robot dynamics model for MPC prediction
 */
void ConvexMPC::getDynamics(scalar_t time_cur, size_t k,
                            const std::shared_ptr<ModeSchedule> mode_schedule) {
  const scalar_t time_k = time_cur + k * dt_;  // 预测时刻 / Prediction time
  
  // 获取参考轨迹 / Get reference trajectories
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto foot_traj = referenceBuffer_->getFootPosTraj();

  // 计算当前步态相位和接触状态 / Calculate current gait phase and contact state
  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  const size_t nf = foot_names.size();
  rcpputils::assert_true(nf == contact_flag.size());

  // 获取当前机器人基座位姿 / Get current robot base pose
  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  vector3_t rpy_des = rpy_traj->evaluate(time_k);  // 期望姿态角 / Desired orientation

  // 计算旋转矩阵和惯性张量 / Calculate rotation matrix and inertia tensor
  matrix_t Rt = toRotationMatrix(rpy_des);
  matrix_t Ig_t = Rt * Ig_ * Rt.transpose();  // 转换到世界坐标系的惯性张量 / Inertia tensor in world frame

  // 构建离散时间线性动力学模型 A*x[k+1] = A*x[k] + B*u[k] + b
  // Build discrete-time linear dynamics model A*x[k+1] = A*x[k] + B*u[k] + b
  // 状态向量: x = [位置, 速度, 姿态角, 角速度]^T / State vector: x = [position, velocity, orientation, angular_velocity]^T
  ocp_[k].A.setIdentity(12, 12);
  ocp_[k].A.block<3, 3>(0, 3).diagonal().fill(dt_);  // 位置积分 / Position integration
  ocp_[k].A.block<3, 3>(6, 9) = dt_ * getJacobiFromOmegaToRPY(rpy_des);  // 姿态积分 / Orientation integration
  
  // 控制输入矩阵B：将接触力映射到状态变化 / Control input matrix B: map contact forces to state changes
  ocp_[k].B.setZero(12, nf * 3);
  
  // 计算质心位置（在参考轨迹和当前位置间插值）/ Calculate center of mass position (interpolate between reference and current)
  vector3_t xc = phase * pos_traj->evaluate(time_k) +
                 (1.0 - phase) * base_pose.translation();
  
  // 为每个足端设置控制输入映射 / Set control input mapping for each foot
  for (size_t i = 0; i < nf; i++) {
    const auto &foot_name = foot_names[i];
    if (contact_flag[i]) {  // 仅处理接触的足端 / Only process contacting feet
      vector3_t pf_i = foot_traj[foot_name]->evaluate(time_k);  // 足端位置 / Foot position
      
      // 线性动量方程：F -> 加速度 / Linear momentum equation: F -> acceleration
      ocp_[k].B.middleRows(3, 3).middleCols(3 * i, 3).diagonal().fill(
          dt_ / total_mass_);
      
      // 角动量方程：(r × F) -> 角加速度 / Angular momentum equation: (r × F) -> angular acceleration
      // 使用足端到质心的距离向量计算力矩臂 / Use foot-to-CoM distance vector to calculate moment arm
      ocp_[k].B.bottomRows(3).middleCols(3 * i, 3) =
          Ig_t.inverse() * skew(dt_ * (pf_i - xc));
    }
  }

  // 设置常数项（重力影响）/ Set constant term (gravity effect)
  ocp_[k].b.setZero(12);
  ocp_[k].b(5) += -dt_ * grav_;  // 重力加速度对z方向速度的影响 / Gravity acceleration effect on z-direction velocity
}

void ConvexMPC::getInequalityConstraints(
    size_t k, size_t N, const std::shared_ptr<ModeSchedule> mode_schedule) {
  const size_t nf = foot_names.size();
  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));

  if (k < N) {
    scalar_t mu = 1 / mu_;
    matrix_t Ci(5, 3);
    Ci << mu, 0, 1., -mu, 0, 1., 0, mu, 1., 0, -mu, 1., 0, 0, 1.;
    ocp_[k].C = matrix_t::Zero(5 * nf, 12);
    ocp_[k].D.setZero(5 * nf, 3 * nf);
    ocp_[k].lg.setZero(5 * nf);
    ocp_[k].ug.setZero(5 * nf);
    ocp_[k].lg_mask.setOnes(5 * nf);
    ocp_[k].ug_mask.setOnes(5 * nf);

    for (size_t i = 0; i < nf; i++) {
      ocp_[k].D.block<5, 3>(i * 5, i * 3) = Ci;
      ocp_[k].ug(5 * i + 4) = contact_flag[i] ? 400 : 0.0;
      ocp_[k].ug_mask.segment(5 * i, 4).setZero();
    }
  }
  /* std::cout << "\n############### " << k << " constraints ################\n"
           << cstr_k; */
}

void ConvexMPC::getCosts(scalar_t time_cur, size_t k, size_t N,
                         const std::shared_ptr<ModeSchedule> mode_schedule) {
  const size_t nf = foot_names.size();
  const scalar_t time_k = time_cur + (k + 1) * dt_;
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();

  vector3_t rpy_des =
      rpy_traj->evaluate(time_k) - rpy_traj->evaluate(time_cur) + rpy_des_start;
  vector3_t omega_des =
      getJacobiFromRPYToOmega(rpy_des) * rpy_traj->derivative(time_k, 1);
  vector3_t v_des = pos_traj->derivative(time_k, 1);

  vector_t x_des(12);
  x_des << pos_traj->evaluate(time_k), v_des, rpy_des, omega_des;

  // std::cout << "xdes " << k << ": " << x_des.transpose() << "\n";

  ocp_[k].Q = weight_;
  ocp_[k].S = matrix_t::Zero(3 * nf, 12);
  ocp_[k].q = -weight_ * x_des;
  ocp_[k].r.setZero(3 * nf);
  if (k < N) {
    ocp_[k].R = 1e-5 * matrix_t::Identity(3 * nf, 3 * nf);
    scalar_t phase = k * dt_ / mode_schedule->duration();
    auto contact_flag =
        biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
    int nc = 0;
    for (bool flag : contact_flag) {
      if (flag) {
        nc++;
      }
    }
    nc = max(1, nc);
    vector3_t force_des_i =
        total_mass_ / nc *
        (vector3_t(0, 0, grav_) + pos_traj->derivative(time_k, 2));
    vector_t force_des = vector_t::Zero(3 * nf);
    for (size_t k = 0; k < nf; k++) {
      if (contact_flag[k]) {
        force_des.segment(3 * k, 3) = force_des_i;
      }
    }
    ocp_[k].r = -ocp_[k].R * force_des;
  } else {
    // ocp_[k].Q = 1e2 * ocp_[k].Q;
    // ocp_[k].q = 1e2 * ocp_[k].q;
  }
}

void ConvexMPC::optimize() {
  // RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"),
  //             "ConvexMPC: generateTrajRef start");
  const scalar_t time_cur = nodeHandle_->now().seconds();
  auto mode_schedule = referenceBuffer_->getModeSchedule();
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();

  if (referenceBuffer_->getFootPosTraj().empty() || pos_traj == nullptr ||
      mode_schedule == nullptr) {
    return;
  }

  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();

  size_t N = mode_schedule->duration() / dt_;
  ocp_.resize(N + 1);

  matrix_t Ig_0 = pinocchioInterface_ptr_->getData().Ig.inertia().matrix();

  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  Ig_ = base_pose.rotation().transpose() * Ig_0 * base_pose.rotation();

  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
  auto rpy_m = toEulerAngles(base_pose.rotation());
  rpy_des_start =
      rpy_m - computeEulerAngleErr(rpy_m, rpy_traj->evaluate(time_cur));

  for (size_t k = 0; k <= N; k++) {
    if (k < N) {
      getDynamics(time_cur, k, mode_schedule);
    }
    getInequalityConstraints(k, N, mode_schedule);
    getCosts(time_cur, k, N, mode_schedule);
  }

  if (solution_.size() == N + 1) {
    solver_settings.warm_start = 1;
  } else {
    solver_settings.warm_start = 0;
    solution_.resize(N + 1);
  }
  hpipm::OcpQpIpmSolver solver(ocp_, solver_settings);

  vector_t x0(12);
  x0 << base_pose.translation(), base_twist.linear(), rpy_m,
      base_twist.angular();

  const auto res = solver.solve(x0, ocp_, solution_);
  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    // for (size_t i = 0; i < solution_.size(); i++) {
    //   std::cout << "x " << i << ": " << solution_[i].x.transpose() << "\n";
    // }
    // for (size_t i = 0; i < solution_.size(); i++) {
    //   std::cout << "u " << i << ": " << solution_[i].u.transpose() << "\n";
    // }
    fitTraj(time_cur, N);
  } else {
    std::cout << "ConvexMPC: " << res << "\n";
    exit(0);
  }
  // RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"),
  //             "ConvexMPC: generateTrajRef done");
}

void ConvexMPC::fitTraj(scalar_t time_cur, size_t N) {
  std::vector<scalar_t> time_array;
  std::vector<vector_t> base_pos_array;
  std::vector<vector_t> base_vel_array;
  std::vector<vector_t> base_rpy_array;
  std::vector<vector_t> base_omega_array;
  std::vector<vector_t> force_array;

  matrix_t P(6, 12);
  P.setZero();
  P.topRows(3).middleCols(3, 3).setIdentity();
  P.bottomRows(3).middleCols(9, 3).setIdentity();
  matrix_t A = 1.0 / dt_ * (ocp_[0].A - matrix_t::Identity(12, 12));
  matrix_t B = 1.0 / dt_ * ocp_[0].B;
  vector_t drift = 1.0 / dt_ * ocp_[0].b;
  vector_t acc_des = P * (A * solution_[0].x + B * solution_[0].u + drift);

  for (size_t k = 0; k < N; k++) {
    time_array.emplace_back(time_cur + k * dt_);
    base_pos_array.emplace_back(solution_[k].x.head(3));
    base_vel_array.emplace_back(solution_[k].x.segment(3, 3));
    base_rpy_array.emplace_back(solution_[k].x.segment(6, 3));
    base_omega_array.emplace_back(solution_[k].x.tail(3));
    force_array.emplace_back(solution_[k].u);
  }
  auto base_pos_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_pos_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      solution_[1].x.segment(3, 3),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      solution_.back().x.segment(3, 3));
  base_pos_traj_ptr_->fit(time_array, base_pos_array);
  referenceBuffer_->setOptimizedBasePosTraj(base_pos_traj_ptr_);

  auto base_vel_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_vel_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[1].x.segment(3, 3) - solution_[0].x.segment(3, 3)),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ *
          (solution_[N - 1].x.segment(3, 3) -
           solution_[N - 2].x.segment(3, 3)));
  base_vel_traj_ptr_->fit(time_array, base_vel_array);
  referenceBuffer_->setOptimizedBaseVelTraj(base_vel_traj_ptr_);

  auto base_rpy_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline_hermite);
  base_rpy_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[1].x.segment(6, 3) - solution_[0].x.segment(6, 3)),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ *
          (solution_[N - 1].x.segment(6, 3) -
           solution_[N - 2].x.segment(6, 3)));
  base_rpy_traj_ptr_->fit(time_array, base_rpy_array);
  referenceBuffer_->setOptimizedBaseRpyTraj(base_rpy_traj_ptr_);

  auto base_omega_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_omega_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[1].x.tail(3) - solution_[0].x.tail(3)),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[N - 1].x.tail(3) - solution_[N - 2].x.tail(3)));
  base_omega_traj_ptr_->fit(time_array, base_omega_array);
  referenceBuffer_->setOptimizedBaseOmegaTraj(base_omega_traj_ptr_);

  auto force_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      foot_names.size() * 3, CubicSplineInterpolation::SplineType::cspline);
  force_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector_t::Zero(foot_names.size() * 3),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector_t::Zero(foot_names.size() * 3));
  force_traj_ptr_->fit(time_array, force_array);
  referenceBuffer_->setOptimizedForceTraj(force_traj_ptr_);
}
} // namespace clear
