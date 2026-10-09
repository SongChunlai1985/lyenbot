#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// ==========================
// 小工具：角度向量（pitch, roll, yaw）
// ==========================
struct PRY {
  double pitch; // 绕Y
  double roll;  // 绕X
  double yaw;   // 绕Z
};

// ==========================
// 生成 Rx/Ry/Rz（右手系）
// ==========================
static inline cv::Matx33d Rx(double a) {
  double c = std::cos(a), s = std::sin(a);
  return cv::Matx33d(1, 0, 0, 0, c, -s, 0, s, c);
}
static inline cv::Matx33d Ry(double a) {
  double c = std::cos(a), s = std::sin(a);
  return cv::Matx33d(c, 0, s, 0, 1, 0, -s, 0, c);
}
static inline cv::Matx33d Rz(double a) {
  double c = std::cos(a), s = std::sin(a);
  return cv::Matx33d(c, -s, 0, s, c, 0, 0, 0, 1);
}

// ==========================
// 你的 MATLAB: Rxyz_zyx(r,p,y) = Rz(y)*Ry(p)*Rx(r)
// 注意：输入顺序我们按 (pitch, roll, yaw) 进来，但这里要按 roll/pitch/yaw组装
// ==========================
static inline cv::Matx33d R_zyx_from_rpy(double roll, double pitch,
                                         double yaw) {
  return Rz(yaw) * Ry(pitch) * Rx(roll);
}

// ==========================
// 旋转误差：rotErrVec(R) 的 C++ 版本
// 返回“旋转向量”的范数（角度大小），越小越接近
// 用 OpenCV Rodrigues 更稳：R -> rvec
// ==========================
static inline double rotErrorAngle(const cv::Matx33d &R) {
  cv::Mat Rm(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      Rm.at<double>(i, j) = R(i, j);

  cv::Mat rvec;
  cv::Rodrigues(Rm, rvec); // rvec = 轴角向量，范数=旋转角
  cv::Vec3d v = rvec;
  return cv::norm(v);
}

// ==========================
// 目标函数：给 x=[roll2, pitch2, yaw2]，计算 Stick2 与 Rdes 的姿态差异
// 你 MATLAB: cost = ||rotErrVec(R2' * Rdes)||^2
// ==========================
static inline double costTilted(const cv::Vec3d &x, const cv::Matx33d &Rdes,
                                double tilt_rad) {
  double r2 = x[0], p2 = x[1], y2 = x[2];

  cv::Matx33d Rtilt = Rx(tilt_rad);

  // Stick2: R2 = Rz(y2)*Rx(r2) * (Rtilt * Ry(p2) * Rtilt')
  cv::Matx33d R2 = (Rz(y2) * Rx(r2)) * (Rtilt * Ry(p2) * Rtilt.t());

  // 误差旋转：Rerr = R2' * Rdes
  cv::Matx33d Rerr = R2.t() * Rdes;

  double ang = rotErrorAngle(Rerr);
  return ang * ang; // 对应 MATLAB 的 ^2
}

// ==========================
// 轻量 Nelder–Mead（类似 fminsearch），3维
// - maxIter: 最大迭代
// - tol: 收敛阈值（单纯形尺寸 + 目标变化）
// ==========================
static cv::Vec3d nelderMead3(const std::function<double(const cv::Vec3d &)> &f,
                             cv::Vec3d x0, int maxIter = 200,
                             double tol = 1e-12) {
  // 单纯形 4 个点（3维 + 1）
  cv::Vec3d x[4];
  x[0] = x0;

  // 初始步长：按角度量级给个小扰动（你也可以改）
  const double step = 1e-3;
  x[1] = x0 + cv::Vec3d(step, 0, 0);
  x[2] = x0 + cv::Vec3d(0, step, 0);
  x[3] = x0 + cv::Vec3d(0, 0, step);

  double fx[4];
  for (int i = 0; i < 4; i++)
    fx[i] = f(x[i]);

  auto sortSimplex = [&]() {
    // 按 fx 从小到大排序（0最好，3最差）
    int idx[4] = {0, 1, 2, 3};
    std::sort(idx, idx + 4, [&](int a, int b) { return fx[a] < fx[b]; });
    cv::Vec3d xs[4];
    double fs[4];
    for (int i = 0; i < 4; i++) {
      xs[i] = x[idx[i]];
      fs[i] = fx[idx[i]];
    }
    for (int i = 0; i < 4; i++) {
      x[i] = xs[i];
      fx[i] = fs[i];
    }
  };

  sortSimplex();

  // 系数（经典 NM）
  const double alpha = 1.0; // 反射
  const double gamma = 2.0; // 扩张
  const double rho = 0.5;   // 收缩
  const double sigma = 0.5; // 缩小

  for (int it = 0; it < maxIter; ++it) {
    // 收敛判据：单纯形尺寸 + 函数值跨度
    double fspan = std::abs(fx[3] - fx[0]);
    double size = 0.0;
    for (int i = 1; i < 4; i++)
      size = std::max(size, cv::norm(x[i] - x[0]));
    if (size < tol && fspan < tol)
      break;

    // 质心（不含最差点 x[3]）
    cv::Vec3d c = (x[0] + x[1] + x[2]) * (1.0 / 3.0);

    // 反射
    cv::Vec3d xr = c + alpha * (c - x[3]);
    double fr = f(xr);

    if (fr < fx[0]) {
      // 扩张
      cv::Vec3d xe = c + gamma * (xr - c);
      double fe = f(xe);
      if (fe < fr) {
        x[3] = xe;
        fx[3] = fe;
      } else {
        x[3] = xr;
        fx[3] = fr;
      }
    } else if (fr < fx[2]) {
      // 接受反射
      x[3] = xr;
      fx[3] = fr;
    } else {
      // 收缩（外/内）
      cv::Vec3d xc;
      if (fr < fx[3]) {
        // 外收缩
        xc = c + rho * (xr - c);
      } else {
        // 内收缩
        xc = c - rho * (c - x[3]);
      }
      double fc = f(xc);

      if (fc < fx[3]) {
        x[3] = xc;
        fx[3] = fc;
      } else {
        // 缩小：除最好点外全部向最好点靠拢
        for (int i = 1; i < 4; i++) {
          x[i] = x[0] + sigma * (x[i] - x[0]);
          fx[i] = f(x[i]);
        }
      }
    }

    sortSimplex();
  }

  return x[0]; // 返回最优点
}

// ==========================
// 核心封装函数：输入 (pitch, roll, yaw) -> 输出 (pitch2, roll2, yaw2)
// - 输入输出单位：弧度
// - tilt_deg：你 MATLAB 里的 12.5 度（默认给 12.5）
// - 初值：直接用输入角（更容易收敛）
// ==========================
PRY fitTiltedAngles_PRY(double pitch, double roll, double yaw,
                        double tilt_deg = 12.5) {
  const double tilt = tilt_deg * M_PI / 180.0;

  // 目标姿态：Stick1 的正常模型
  // MATLAB: Rdes = Rxyz_zyx(r1,p1,y1) = Rz(yaw)*Ry(pitch)*Rx(roll)
  cv::Matx33d Rdes = R_zyx_from_rpy(roll, pitch, yaw);

  // 优化变量顺序：x = [roll2, pitch2, yaw2]
  cv::Vec3d x0(roll, pitch, yaw);

  auto f = [&](const cv::Vec3d &x) -> double {
    return costTilted(x, Rdes, tilt);
  };

  cv::Vec3d xopt = nelderMead3(f, x0, /*maxIter*/ 300, /*tol*/ 1e-12);

  PRY out;
  out.pitch = xopt[1];
  out.roll = xopt[0];
  out.yaw = xopt[2];
  return out;
}
