#include <common.hpp>

namespace utils {

bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<vector<3>> &points, vector<3> &pt_world) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(2 * poses.size(), 4);
  Eigen::Matrix<double, Eigen::Dynamic, 1> b(2 * poses.size());
  b.setZero();
  for (size_t i = 0; i < poses.size(); ++i) {
    matrix<3, 4> m = poses[i].matrix3x4();
    A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
    A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
  }
  auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

  if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
    return true;
  }
  return false;
}
vector<3> inverseDepth(const vector<3> &p, matrix<3> &dip, matrix<3> *ddip) {
  // not really "depth" / distance for Fisheye... could be improved?
  const vector<3> ip = vector<3>(p.x(), p.y(), 1) / p.z();
  dip = matrix<3>::Zero();
  dip.topLeftCorner<2, 2>() = matrix<2>::Identity() / p.z();
  dip.col(2) = -ip / p.z();

  if (ddip != nullptr) {
    const double ipz2 = 1.0 / (p.z() * p.z());
    for (int i = 0; i < 3; ++i) {
      matrix<3> &ddip_i = ddip[i];
      ddip_i.setZero();
      if (i < 2) {
        vector<3> dp = vector<3>::Zero();
        dp(i) = 1;
        ddip_i.col(2) = -dp * ipz2;
      } else {
        ddip_i.topLeftCorner<2, 2>() = -matrix<2>::Identity() * ipz2;
        ddip_i.col(2) = 2 * ip * ipz2;
      }
    }
  }

  return ip;
}
bool triangulateStereoFeatureIdp(const std::vector<SE3> &cam_poses,
                                 const std::vector<vector<3>> &points,
                                 vector<3> &triangulatedPointIdp,
                                 matrix<3> *triangulatedCov) {

  // Implements this triangulation method: (w)Mid2
  // https://bmvc2019.org/wp-content/uploads/papers/0331-paper.pdf

  // Here the formulas are interpreted so that 0 in the paper means second
  // camera and 1 means the first camera, that is, the meaning of 0 and 1
  // are flipped compared to triangulateStereoLinear & mid-point

  const matrix<4> secondToFirstCamera =
      (cam_poses[1] * cam_poses[0].inverse()).matrix();
  const vector<3> f0 = points[0];
  const vector<3> f1 = points[1];

  const vector<3> f0hat = f0.stableNormalized();
  const vector<3> f1hat = f1.stableNormalized();

  const matrix<3> R = secondToFirstCamera.topLeftCorner<3, 3>();
  const vector<3> t = secondToFirstCamera.block<3, 1>(0, 3);

  const vector<3> p = (R * f0hat).cross(f1hat);
  const vector<3> q = (R * f0hat).cross(t);
  const vector<3> r = f1hat.cross(t);
  const double pn = p.norm();
  const double qn = q.norm();
  const double rn = r.norm();

  const double lambda0 = rn / (pn + 1e-5);
  const double lambda1 = qn / (pn + 1e-5);
  const double w = qn / (qn + rn + 1e-5);
  const vector<3> pf = w * (t + lambda0 * (R * f0hat + f1hat));

  const vector<3> l0Rf0hat = lambda0 * R * f0hat;
  const vector<3> l1f1hat = lambda1 * f1hat;

  const double c0 = (t + l0Rf0hat - l1f1hat).squaredNorm(),
               c1 = (t + l0Rf0hat + l1f1hat).squaredNorm(),
               c2 = (t - l0Rf0hat - l1f1hat).squaredNorm(),
               c3 = (t - l0Rf0hat + l1f1hat).squaredNorm();

  if (c0 > std::min(std::min(c1, c2), c3)) {
    return false;
  } else if (!triangulatedCov) {
    triangulatedPointIdp = vector<3>(pf.x(), pf.y(), 1) / pf.z();
    return true;
  }

  // sensitivity
  matrix<3> dpf0hat, dpf1hat;
  for (int i = 0; i < 3; ++i) {
    // d?i = df?hat_i
    vector<3> d0i = vector<3>::Zero(), d1i = vector<3>::Zero();
    d0i(i) = 1;
    d1i(i) = 1;

    const vector<3> dp_d0i = (R * d0i).cross(f1hat);
    const vector<3> dp_d1i = (R * f0hat).cross(d1i);
    const vector<3> dq_d0i = (R * d0i).cross(t);
    const vector<3> dr_d1i = d1i.cross(t);

    const double dpn_d0i = dp_d0i.dot(p) / (pn + 1e-5);
    const double dpn_d1i = dp_d1i.dot(p) / (pn + 1e-5);
    const double dqn_d0i = dq_d0i.dot(q) / (qn + 1e-5);
    const double drn_d1i = dr_d1i.dot(r) / (qn + 1e-5);

    const double dlambda0_d0i = -lambda0 / (pn + 1e-5) * dpn_d0i;
    const double dlambda0_d1i =
        -lambda0 / (pn + 1e-5) * dpn_d1i + drn_d1i / (pn + 1e-5);

    // w = (qn / (qn + rn))
    const double dw_d0i =
        dqn_d0i / (qn + rn + 1e-5) - w / (qn + rn + 1e-5) * dqn_d0i;
    const double dw_d1i = -w / (qn + rn + 1e-5) * drn_d1i;

    const vector<3> vInner = R * f0hat + f1hat;
    dpf0hat.col(i) = dw_d0i * (t + lambda0 * vInner) +
                     w * dlambda0_d0i * vInner + w * lambda0 * R * d0i;
    dpf1hat.col(i) = dw_d1i * (t + lambda0 * vInner) +
                     w * dlambda0_d1i * vInner + w * lambda0 * d1i;
  }

  const matrix<3> df0 =
      (matrix<3>::Identity() - f0hat * f0hat.transpose()) / (f0.norm() + 1e-5);
  const matrix<3> df1 =
      (matrix<3>::Identity() - f1hat * f1hat.transpose()) / (f1.norm() + 1e-5);

  const matrix<3, 2> dpf_df0 = dpf0hat * df0.topLeftCorner<3, 2>();
  const matrix<3, 2> dpf_df1 = dpf1hat * df1.topLeftCorner<3, 2>();

  // Inverse depth parametrization
  matrix<3> dpfi_dpf;
  triangulatedPointIdp = inverseDepth(pf, dpfi_dpf, nullptr);
  const matrix<3, 2> dpfi_df0 = dpfi_dpf * dpf_df0;
  const matrix<3, 2> dpfi_df1 = dpfi_dpf * dpf_df1;

  // assuming isotropic error in normalized pixel coordinates (TODO: this can be
  // improved)
  *triangulatedCov =
      dpfi_df0 * dpfi_df0.transpose() + dpfi_df1 * dpfi_df1.transpose();

  return true;
}

} // namespace utils