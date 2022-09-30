#ifndef G2O_TYPES
#define G2O_TYPES
// g2o components
#include <common.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace my_slam {
class VertexPose : public g2o::BaseVertex<6, SE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override { _estimate = SE3(); }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    vector<6> update_eigen;
    (update_eigen << update[0], update[1], update[2], update[3], update[4],
     update[5])
        .finished();
    _estimate = SE3::exp(update_eigen) * _estimate;
  }

  virtual bool read(std::istream &in) override { return true; }

  virtual bool write(std::ostream &out) const override { return true; }
};

class EdgeProjectionPoseOnly
    : public g2o::BaseUnaryEdge<2, vector<2>, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjectionPoseOnly(const vector<3> &pos, const matrix<3> &K)
      : _pos3d(pos), _K(K) {}

  virtual void computeError() override {
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    SE3 T = v->estimate();
    vector<3> pos_pixel = _K * (T * _pos3d);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  virtual void linearizeOplus() override {
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    SE3 T = v->estimate();
    vector<3> pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    (_jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
     -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
     fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv)
        .finished();
  }

  virtual bool read(std::istream &in) override { return true; }

  virtual bool write(std::ostream &out) const override { return true; }

private:
  vector<3> _pos3d;
  matrix<3> _K;
};

class VertexXYZ : public g2o::BaseVertex<3, vector<3>> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void setToOriginImpl() override { _estimate = vector<3>::Zero(); }

  virtual void oplusImpl(const double *update) override {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
  }

  virtual bool read(std::istream &in) override { return true; }

  virtual bool write(std::ostream &out) const override { return true; }
};

class EdgeProjection
    : public g2o::BaseBinaryEdge<2, vector<2>, VertexPose, VertexXYZ> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjection(const matrix<3> &K, const SE3 &cam_ext)
      : _K(K), _cam_ext(cam_ext) {}

  virtual void computeError() override {
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    SE3 T = v0->estimate();
    vector<3> pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  virtual void linearizeOplus() override {
    // std::cout << "linearizeOplus" << std::endl;
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    SE3 T = v0->estimate();
    vector<3> pw = v1->estimate();
    vector<3> pos_cam = _cam_ext * T * pw;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    (_jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
     -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
     fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv)
        .finished();

    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                       _cam_ext.rotationMatrix() * T.rotationMatrix();
  }

  virtual bool read(std::istream &in) override { return true; }

  virtual bool write(std::ostream &out) const override { return true; }

private:
  matrix<3> _K;
  SE3 _cam_ext;
};
} // namespace my_slam
#endif