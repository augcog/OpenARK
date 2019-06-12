/** SR-UKF with support for Quaternions
  * From https://github.com/sxyu/Quaternion-SR-UKF
  * Some parts of this implementation are based on https://github.com/sfwa/ukf. 
  * The goals of this implementation (compared to sfwa/ukf) are:
  * - no dependency on C++14 features for compatibility reasons
  * - additional features for avatar tracking
  * - more concise and efficient, by grouping similar types of state variables allowing for vectorized computations.
  */
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

namespace ark {
    namespace kalman {
        namespace util {
            /** convert a rotation vector to a quaternion */
            template<class Params>
            Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d & r) {
                double x_2 = r.squaredNorm();
                double d_q_w = (-Params::MRP_A * x_2 + Params::MRP_F
                    * std::sqrt(x_2 * (double(1.0) - Params::MRP_A*Params::MRP_A)
                        + Params::MRP_F * Params::MRP_F))
                    / (Params::MRP_F * Params::MRP_F + x_2);
                Eigen::Vector3d d_q_xyz = r * (d_q_w + Params::MRP_A) * (double(1.0) / Params::MRP_F);
                Eigen::Quaterniond d_q;
                d_q.vec() = d_q_xyz;
                d_q.w() = d_q_w;
                return d_q;
            }

            //inline Eigen::Quaterniond quaternionDelta(const Eigen::Quaterniond & q, const Eigen::Vector3d & omega) {
            //    Eigen::Quaterniond omega_q;
            //    omega_q.vec() = omega;
            //    omega_q.w() = 0;
            //    return omega_q.conjugate() * q;
            //}

            /** auto compute derivatives of all quaternions, given that ang. vel.s are stored at vecs3d[omega_start:] */
            template<class VecType>
            void diffQuaternion(VecType & in_out, int omega_start = 0) {
                for (int i = 0; i < in_out._NUM_QUATERNIONS; ++i) {
                    auto temp_q = in_out.quat(i);
                    temp_q.vec() = in_out.template segment<3>(omega_start + 3 * i);
                    temp_q.w() = 0;
                }
                in_out.template segment<3 * in_out._NUM_QUATERNIONS>(omega_start).setZero();
            }

            /** auto compute derivatives of all position vectors in interval,
              * position variables are in interval [p_start, p_end)
              * corresponding velocities are in interval [v_start, v_start + p_end - p_start)  */
            template<class VecType>
            void diffPosition(VecType & in_out, int p_start, int p_end, int v_start) {
                in_out.segment(p_start, p_end - p_start) = in_out.segment(v_start, p_end - p_start);
                in_out.segment(v_start, p_end - p_start).setZero();
            }

            /** Numerical integrators */
            namespace integrator {
                /* Fourth-order Runge-Kutta integrator (the default method) */
                class RK4 {
                public:
                    template <class Derivative, class InputType>
                    static typename Derivative::StateVec integrate(double delta, const typename Derivative::StateVec & state,
                                                                   const InputType & u) {
                        typename Derivative::StateVec a = Derivative::dF(state, u);
                        typename Derivative::StateVec b = Derivative::dF(typename Derivative::StateVec(state + 0.5 * delta * a), u);
                        typename Derivative::StateVec c = Derivative::dF(typename Derivative::StateVec(state + 0.5 * delta * b), u);
                        typename Derivative::StateVec d = Derivative::dF(typename Derivative::StateVec(state + delta * c), u);
                        return state + (delta / 6.0) * (a + (b * 2.0) + (c * 2.0) + d);
                    }
                };

                /* Second-order Heun integrator */
                class Heun {
                public:
                    template <class Derivative, class InputType>
                    static typename Derivative::StateVec integrate(double delta, const typename Derivative::StateVec& state,
                                                                   const InputType & u) {
                        typename Derivative::StateVec initial = Derivative::dF(state, u);
                        typename Derivative::StateVec predictor = state + delta * initial;
                        return state + (delta * 0.5) * (initial + Derivative::dF(predictor, u));
                    }
                };

                /* First-order Euler integrator */
                class Euler {
                    public:
                    template <class Derivative, class InputType>
                    static typename Derivative::StateVec integrate(double delta, const typename Derivative::StateVec& state,
                                                                   const InputType & u) {
                        return state + delta * Derivative::dF(state, u);
                    }
                };
            }
        }

        /** KF state/measurement vector representation
          * vector is segmented into: scalars (1), 3-vectors (3), quaternions(4)
          * uses the specified integrator to integrate
          */
        template<int NUM_SCALARS, int NUM_3D_VECS = 0, int NUM_QUATERNIONS = 0>
        class Vector : public Eigen::Matrix<double, NUM_SCALARS + NUM_3D_VECS * 3 + NUM_QUATERNIONS * 4, 1>  {
        public:
            typedef Eigen::Matrix<double, NUM_SCALARS + NUM_3D_VECS * 3 + NUM_QUATERNIONS * 4, 1> Base;
            using Base::Base;
            using Base::operator=;

            static const int _NUM_SCALARS = NUM_SCALARS;
            static const int _NUM_3D_VECS = NUM_3D_VECS;
            static const int _NUM_QUATERNIONS = NUM_QUATERNIONS;

            /** size of vector */
            static const int SIZE             = NUM_QUATERNIONS * 4 + NUM_3D_VECS * 3 + NUM_SCALARS;
            /** num covariance parameters */
            static const int COV_SIZE         = NUM_QUATERNIONS * 3 + NUM_3D_VECS * 3 + NUM_SCALARS;
            /** num sigma points */
            static const int NUM_SIGMA_POINTS = COV_SIZE * 2 + 1;
            
            /** type shorthands */
            // delta: for updates, which should be COV_SIZE in length (3 parameters only for each quaternion)
            typedef Eigen::Matrix<double, COV_SIZE, 1> VectorDelta;

            // maps to parts of the vector
            typedef Eigen::Matrix<double, NUM_SCALARS, 1> ScalarVector;
            typedef Eigen::Matrix<double, 3, NUM_3D_VECS> SpatialVector;
            typedef Eigen::Matrix<double, 4, NUM_QUATERNIONS> QuatVector;

            /** get map to all quaternions (unfortunately, as 4xn matrix; see quat()) */
            Eigen::Map<QuatVector> quats() {
                Eigen::Map<QuatVector> result(this->data() + QUAT_START);
                return result;
            }
            /** get map to all quaternions (unfortunately, as 4xn matrix; see quat()) */
            const Eigen::Map<QuatVector> quats() const {
                Eigen::Map<const QuatVector> result(this->data() + QUAT_START);
                return result;
            }
            /** get map to all 3d vectors (as 3xn matrix) */
            Eigen::Map<SpatialVector> vecs3d() {
                Eigen::Map<SpatialVector> result(this->data() + VECS_START);
                return result;
            }
            /** get map to all 3d vectors (as 3xn matrix) */
            const Eigen::Map<const SpatialVector> vecs3d() const {
                Eigen::Map<const SpatialVector> result(this->data() + VECS_START);
                return result;
            }
            /** map to all scalars */
            Eigen::Map<ScalarVector> scalars() {
                Eigen::Map<ScalarVector> result(this->data() + SCALAR_START);
                return result;
            }
            /** map to all scalars */
            const Eigen::Map<const ScalarVector> scalars() const {
                Eigen::Map<const ScalarVector> result(this->data() + SCALAR_START);
                return result;
            }
            /* get a quaternion parameter at the given index (quat.col(idx), but as a quaternion) */
            Eigen::Map<Eigen::Quaterniond> quat(int idx) {
                Eigen::Map<Eigen::Quaterniond> quatMap(this->data() + QUAT_START + 4 * idx);
                return quatMap;
            }

            /* get a quaternion parameter at the given index (quat.col(idx), but as a quaternion) */
            const Eigen::Map<const Eigen::Quaterniond> quat(int idx) const {
                Eigen::Map<const Eigen::Quaterniond> quatMap(this->data() + QUAT_START + 4 * idx);
                return quatMap;
            }

            /** Update the state vector using a delta vector,
             * where rodrigues vectors are computed using the given params. */
            template<class Params>
            void applyDelta(const VectorDelta & delta) {
                this->template head<QUAT_START>() += delta.template head<QUAT_START>();
                for (int i = 0; i < NUM_QUATERNIONS; ++i) {
                    quat(i) = kalman::util::rotationVectorToQuaternion<Params>(delta.template segment<3>(QUAT_START + 3 * i))
                        * quat(i);
                }
            }

            /** Compute innovation: actual - predicted difference */
            static Vector innovation(const Vector & actual, const Vector & predict) {
                Vector sv;
                sv.head(QUAT_START) = actual.head(QUAT_START) - predict.head(QUAT_START);
                /** for quaternions, use rotation between */
                for (int i = 0; i < NUM_QUATERNIONS; ++i) {
                    sv.quat(i) = actual.quat(i) * predict.quat(i).inverse();
                }
                return sv;
            }

            /** start positions of data blocks */
            static const int SCALAR_START = 0;
            static const int VECS_START = NUM_SCALARS;
            static const int QUAT_START = NUM_SCALARS + NUM_3D_VECS * 3;
        }; // Vector
        
        // required namespace-scope declarations to avoid linker error
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::_NUM_SCALARS;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::_NUM_3D_VECS;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::_NUM_QUATERNIONS;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::SIZE;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::COV_SIZE;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::NUM_SIGMA_POINTS;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::SCALAR_START;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::VECS_START;
        template<int NUM_SCALARS, int NUM_3D_VECS, int NUM_QUATERNIONS>
            const int Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>::QUAT_START;

        /** Methods for performing unscented transform and manipulating sigma points */
        namespace unscented {
            /** perform unscented transform: computes sigma points from the input Gaussian */
            template<class Params, class VecType>
            Eigen::MatrixXd transform(VecType & mean, const Eigen::MatrixXd & S) {
                Eigen::MatrixXd X(VecType::SIZE, VecType::NUM_SIGMA_POINTS);
                auto vecs = mean.template head<VecType::QUAT_START>();
                auto covs = S.template topLeftCorner<VecType::QUAT_START, VecType::COV_SIZE>();
                X.template topLeftCorner<VecType::QUAT_START, 1>() = vecs;
                X.template block<VecType::QUAT_START, VecType::COV_SIZE>(0, 1) = covs.colwise() + vecs;
                X.template block<VecType::QUAT_START, VecType::COV_SIZE>(0, VecType::COV_SIZE + 1) = -(covs.colwise() - vecs);

                for (int i = 0; i < VecType::_NUM_QUATERNIONS; ++i) {
                    auto out = X.template block<4, VecType::NUM_SIGMA_POINTS>(VecType::QUAT_START + 4 * i, 0);
                    auto state = mean.quat(i);
                    auto cov = S.template block<3, VecType::COV_SIZE>(VecType::QUAT_START + 3 * i, 0);
                    out.col(0) << state.vec(), state.w();

                    Eigen::Array<double, 1, VecType::COV_SIZE> x_2 = cov.colwise().squaredNorm();
                    Eigen::Array<double, 1, VecType::COV_SIZE> err_w =
                        (-Params::MRP_A * x_2 + Params::MRP_F
                            * (x_2 * (1.0 - Params::MRP_A * Params::MRP_A)
                                + Params::MRP_F * Params::MRP_F).sqrt())
                        / (Params::MRP_F * Params::MRP_F + x_2);
                    Eigen::Array<double, 3, VecType::COV_SIZE> err_xyz = cov.array().rowwise() * (err_w + Params::MRP_A)
                        * (1.0 / Params::MRP_F);

                    Eigen::Quaterniond temp_q;
                    for (std::size_t i = 0; i < VecType::COV_SIZE; i++) {
                        temp_q = Eigen::Quaterniond(err_w(i), err_xyz(0, i), err_xyz(1, i), err_xyz(2, i)) * state;
                        out.col(i + 1) << temp_q.vec(), temp_q.w();
                    }

                    for (std::size_t i = 0; i < VecType::COV_SIZE; i++) {
                        temp_q = Eigen::Quaterniond(err_w(i), err_xyz(0, i), err_xyz(1, i), err_xyz(2, i)).conjugate() * state;
                        out.col(i + VecType::COV_SIZE + 1) << temp_q.vec(), temp_q.w();
                    }
                }
                return X;
            }

            /** apply a measurement transformation to the Gaussian,
              * provided as class with static method 'H' */
            template<class MeasureDef, class InputType>
            Eigen::MatrixXd applyMeasurement(const Eigen::MatrixXd & X, const InputType & u) {
                Eigen::MatrixXd Xt(MeasureDef::MeasureVec::SIZE, MeasureDef::StateVec::NUM_SIGMA_POINTS);
                for (int i = 0; i < MeasureDef::StateVec::NUM_SIGMA_POINTS; ++i) {
                    Xt.col(i) << MeasureDef::H(X.col(i), u);
                }
                return Xt;
            }

            /** apply a continuous-time process, provided as a derivative class (with static method 'dF'),
              * to the Gaussian, where delta_t is the length of the time step.
              * Integrates numerically with the specified integrator algorithm */
            template<class ProcessDef, class InputType, class Integrator = kalman::util::integrator::RK4>
            void applyProcessInPlace(Eigen::MatrixXd & X, const InputType & u, double delta_t) {
                for (int i = 0; i < ProcessDef::StateVec::NUM_SIGMA_POINTS; ++i) {
                    X.col(i) = Integrator::template integrate<ProcessDef, InputType>(delta_t, typename ProcessDef::StateVec(X.col(i)), u);
                }
            }

            /** apply a continuous-time process, provided as a derivative class (with static method 'dF'),
              * to the Gaussian, where delta_t is the length of the time step.
              * Integrates numerically with the specified integrator algorithm */
            template<class ProcessDef, class InputType, class Integrator = kalman::util::integrator::RK4>
            Eigen::MatrixXd applyProcess(const Eigen::MatrixXd & X, const InputType & u, double delta_t) {
                Eigen::MatrixXd Xt = X;
                applyProcessInPlace<ProcessDef, InputType, Integrator>(Xt, u, delta_t);
                return Xt;
            }

            /** Inverse unscented transform - recover mean of distribution */
            template<class Params, class StateVecType, class SigmaVecType = StateVecType>
            SigmaVecType recoverMean(const Eigen::MatrixXd & X) {
                // simply take the mean of all sigma points
                SigmaVecType mean =
                    Params::sigma_WMI * X.template rightCols<StateVecType::NUM_SIGMA_POINTS - 1>().rowwise().sum()
                    + Params::sigma_WM0 * X.col(0);

                // normalize quaternions
                for (int i = 0; i < SigmaVecType::_NUM_QUATERNIONS; ++i) {
                    mean.quat(i).normalize();
                }
                return mean;
            }

            /** Inverse unscented transform - compute deltas from initial mean */
            template<class Params, class StateVecType, class SigmaVecType = StateVecType>
            Eigen::MatrixXd computeDeltas(Eigen::MatrixXd & X, SigmaVecType & init_mean) {
                // recover 
                Eigen::MatrixXd wPrime(SigmaVecType::COV_SIZE, StateVecType::NUM_SIGMA_POINTS);
                wPrime.template topRows<SigmaVecType::QUAT_START>() = X.template topRows<SigmaVecType::QUAT_START>().colwise() -
                    init_mean.template head<SigmaVecType::QUAT_START>();

                /* calculate quaternion deltas using Kraft, equation 45 */
                for (int i = 0; i < StateVecType::NUM_SIGMA_POINTS; ++i) {
                    for (int j = 0; j < SigmaVecType::_NUM_QUATERNIONS; ++j) {
                        int row_pos_in = SigmaVecType::QUAT_START + 4 * j, row_pos_out = SigmaVecType::QUAT_START + 3 * j;
                        Eigen::Quaterniond delta_q = Eigen::Quaterniond(X.template block<4, 1>(row_pos_in, i)) * init_mean.quat(j).conjugate();
                        wPrime.template block<3, 1>(row_pos_out, i) = Params::MRP_F * delta_q.vec() /
                            (std::abs(Params::MRP_A + delta_q.w()) >
                                std::numeric_limits<double>::epsilon() ?
                                Params::MRP_A + delta_q.w() : std::numeric_limits<double>::epsilon());
                    }
                }
                return wPrime;
            }

            /** Inverse unscented transform - recover sqrt covariance. Note: input here is deltas.
              * Note 2: this is not used at all by the SRUKF. */
            template<class Params, class VecType>
            Eigen::MatrixXd recoverS(Eigen::MatrixXd & wPrime) {
                Eigen::MatrixXd cov;
                /* Calculate the covariance using equation 64 from the Kraft paper. */
                cov = Eigen::MatrixXd::Zero(VecType::COV_SIZE, VecType::COV_SIZE);
                for (int i = 1; i < VecType::NUM_SIGMA_POINTS; i++) {
                    cov.noalias() += Params::sigma_WCI * (wPrime.col(i) * wPrime.col(i).transpose());
                }
                cov.noalias() += Params::sigma_WC0 * (wPrime.col(0) * wPrime.col(0).transpose());
                return cov;
            }
        };

        /** The default parameters for a given UKF model */
        template <class UKFModel>
        struct DefaultUKFParams {
            // parameters
            static constexpr double alphaSquared = 1.0;
            static constexpr double beta = 0.0;
            static constexpr double lambda = alphaSquared * (UKFModel::StateVec::COV_SIZE + 3.0)
                                             - UKFModel::StateVec::COV_SIZE;

            /*
            Definitions for parameters used to calculated MRP vectors.
            See the Markley paper for further details.

            Note: By default, we use Gibbs vectors, which have a singularity at 180
            degrees. This is to avoid numerical issues calculating the covariance
            matrix when the quaternion covariance vector is large and the SUT scaling
            parameter alpha is set very small.

            The singularity being 180 degrees instead of 360 is not a problem unless
            the rotation is expected to change by more than 180 degrees in a single
            filter iteration; if it is, setting the MRP_A parameter to 1.0 moves the
            singularity to 360 degrees.
            */
            static constexpr double MRP_A = 0.0;
            static constexpr double MRP_F = 2.0 * (MRP_A + 1.0);

            /*
            Definitions for sigma point weights. The naming convention follows that used
            in in Van der Merwe 2001.
            */
            static constexpr double sigma_WM0 = lambda / (UKFModel::StateVec::COV_SIZE + lambda);
            static constexpr double sigma_WC0 = sigma_WM0 + (1.0 - alphaSquared + beta);
            static constexpr double sigma_WMI = 1.0 / (2.0 * (UKFModel::StateVec::COV_SIZE + lambda));
            static constexpr double sigma_WCI = sigma_WMI;
        };
        
        // required namespace-scope declarations to avoid linker error
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::alphaSquared;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::beta;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::lambda;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::MRP_A;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::MRP_F;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::sigma_WM0;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::sigma_WC0;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::sigma_WMI;
        template <class UKFModel> constexpr double DefaultUKFParams<UKFModel>::sigma_WCI;

        /** Square Root unscented Kalman Filter for robust pose estimation,
          * with support for quaternions and non-identical time steps
          * UKFModel: class with;
          *  >> TYPEDEFS StateVec, MeasureVec to specify vector formats
          *  >> member FUNCTIONS StateVec dF(StateVec) for finding derivative (process model)
          *                      MeasureVec H(StateVec) to compute measurement for state (measurement model)
          *                      void init(UKF) to initialize state, covariances etc.
          * InputType: type of input for process
          * UKFParam: optionally, class with parameters. 
          * Integrator: optionally, numerical integrator to use. Should be ark::kalman::util::integrator::XYZ
          */
        template<class UKFModel,
                 class InputType = Eigen::MatrixXd,
                 class UKFParams = DefaultUKFParams<UKFModel>,
                 class Integrator = kalman::util::integrator::RK4>
        class UKF {
            typedef typename UKFModel::StateVec StateVec;
            typedef typename UKFModel::MeasureVec MeasureVec;

            // internal storage
            Eigen::MatrixXd sigmaPoints;
            Eigen::MatrixXd wPrime;
            Eigen::MatrixXd zPrime;

        public:

            /** constructor; calls model initializer */
            UKF() { UKFModel::init(*this); }

            /** state mean; please specify this in init */
            StateVec state;

            /** state root covariance; please specify this in init */
            Eigen::MatrixXd stateRootCov;

            /** process noise root covariance (constant); please specify this in init */
            Eigen::MatrixXd processNoiseRootCov;

            /** measurement noise root covariance (constant); please specify this in init (should be diagonal) */
            Eigen::MatrixXd measureNoiseRootCov;

            /** innovation (actual - predicted measurement) mean; calculated internally */
            MeasureVec innovation;
            /** innovation root covariance; calculated internally */
            Eigen::MatrixXd innovationRootCov;

            /** SR-UKF update, adapted from github.com/sfwa/ukf, reference: Van der Merwe 2001 */
            void update(double delta_t, const MeasureVec & z, const InputType & input) {
                // 'a priori' step
                /* calc sigma points */
                sigmaPoints = unscented::template transform<UKFParams, StateVec>(state,
                    Eigen::MatrixXd(stateRootCov * std::sqrt(StateVec::COV_SIZE + UKFParams::lambda)));

                /* Propagate the sigma points through the process model. */
                unscented::applyProcessInPlace<UKFModel, InputType, Integrator>(sigmaPoints, input, delta_t);

                /* Calculate the a priori estimate mean, deltas and root covariance. */
                state = unscented::recoverMean<UKFParams, StateVec>(sigmaPoints);
                wPrime = unscented::computeDeltas<UKFParams, StateVec>(sigmaPoints, state);

                /*
                Create an augmented matrix containing all but the centre sigma point
                delta, with the process noise root covariance to the right.
                */
                Eigen::MatrixXd wPrime_RC(StateVec::COV_SIZE, StateVec::NUM_SIGMA_POINTS - 1 + StateVec::COV_SIZE);
                wPrime_RC << std::sqrt(UKFParams::sigma_WCI) * wPrime.rightCols(StateVec::NUM_SIGMA_POINTS - 1), processNoiseRootCov;

                /*
                Calculate the QR decomposition of the augmented sigma point deltas.
                */
                stateRootCov = wPrime_RC.transpose().householderQr().matrixQR().topLeftCorner(
                    StateVec::COV_SIZE, StateVec::COV_SIZE).template triangularView<Eigen::Upper>();

                /*
                Do a rank-one Cholesky update of the root covariance matrix using the
                central sigma point delta.
                */
                Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(
                    stateRootCov, wPrime.col(0), UKFParams::sigma_WC0);
                stateRootCov.transposeInPlace();

                /* Recalculate the sigma points using the a priori covariance. */
                sigmaPoints = unscented::template transform<UKFParams, StateVec>(state,
                    Eigen::MatrixXd(stateRootCov * std::sqrt(StateVec::COV_SIZE + UKFParams::lambda)));
                wPrime = unscented::computeDeltas<UKFParams, StateVec>(sigmaPoints, state);

                // 'innovation' step
                Eigen::MatrixXd measuredSigmaPoints = unscented::template applyMeasurement<UKFModel, InputType>(sigmaPoints, input);

                MeasureVec zPred = unscented::recoverMean<UKFParams, StateVec, MeasureVec>(measuredSigmaPoints);
                zPrime = unscented::computeDeltas<UKFParams, StateVec, MeasureVec>(measuredSigmaPoints, zPred);
                innovation = MeasureVec::innovation(z, zPred);

                /*
                Create an augmented matrix containing all but the centre innovation
                delta, with the measurement root covariance to the right.
                */
                Eigen::MatrixXd zPrime_RC(z.size(), StateVec::NUM_SIGMA_POINTS - 1 + z.size());
                zPrime_RC.template block<MeasureVec::SIZE, StateVec::NUM_SIGMA_POINTS - 1>(0, 0) =
                    std::sqrt(UKFParams::sigma_WCI) * zPrime.rightCols(StateVec::NUM_SIGMA_POINTS - 1);
                zPrime_RC.template block<MeasureVec::SIZE, MeasureVec::SIZE>(0, StateVec::NUM_SIGMA_POINTS - 1)
                                    = measureNoiseRootCov;

                // Calculate the QR decomposition of the augmented innovationDeltas.
                innovationRootCov =
                    zPrime_RC.transpose().householderQr().matrixQR().template topLeftCorner<MeasureVec::SIZE,
                        MeasureVec::SIZE>().template triangularView<Eigen::Upper>();


                /* Do a rank-one Cholesky update of the innovation root covariance matrix
                   using the central innovationDelta. */
                Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(
                    innovationRootCov, zPrime.col(0), UKFParams::sigma_WC0);
                innovationRootCov.transposeInPlace();

                // 'a posteriori' step

                /*
                Calculate the cross-correlation matrix described in equations 70 and
                71 from from the Kraft paper.
                */
                Eigen::MatrixXd crossCorr = Eigen::MatrixXd::Zero(StateVec::COV_SIZE, MeasureVec::SIZE);
                for (std::size_t i = 1; i < StateVec::NUM_SIGMA_POINTS; i++) {
                    crossCorr.noalias() += UKFParams::sigma_WCI * (wPrime.col(i) * zPrime.col(i).transpose());
                }
                crossCorr.noalias() += UKFParams::sigma_WC0 * (wPrime.col(0) * zPrime.col(0).transpose());

                /*
                Calculate the Kalman gain using QR decomposition. This expression
                implements (S¡¯\(S\P¡¯))¡¯, which is equivalent to (P/S¡¯)/S given in
                literature. Eigen's QR decomposition implements a left-division,
                rather than the right-division assumed in the literature.
                */
                Eigen::MatrixXd kalmanGain =
                    innovationRootCov.transpose().fullPivHouseholderQr().solve(
                    innovationRootCov.fullPivHouseholderQr().solve(
                        crossCorr.transpose())).transpose();

                /*
                Calculate the update delta vector, to be applied to the a priori
                estimate.
                */
                typename StateVec::VectorDelta updateDelta = kalmanGain * innovation;

                // Apply the update delta to the state vector.
                state.template applyDelta<UKFParams>(updateDelta);

                // Reuse the crossCorr variable, since we don't need it again.
                crossCorr.noalias() = kalmanGain * innovationRootCov;

                /*
                Update the root covariance using a series of rank-one Cholesky
                downdates.
                */
                for (std::ptrdiff_t i = 0; i < crossCorr.cols(); i++) {
                    Eigen::internal::llt_inplace<double, Eigen::Lower>::rankUpdate(
                        stateRootCov, crossCorr.col(i), double(-1.0));
                }
            }

            /** initialize state vector to zero (any quaternions states to identity)
              * and all cov matrices to identity, with specified scaling factors*/
            void defaultInitialize(double alphaStateRootCov = 1e-3,
                                   double alphaProcRootCov = 5e-3,
                                   double alphaMeasRootCov = 0.1) {
                state.setZero();
                for (int i = 0; i < state._NUM_QUATERNIONS; ++i) {
                    state.quat(i).setIdentity();
                }

                stateRootCov = Eigen::MatrixXd::Identity(StateVec::COV_SIZE, StateVec::COV_SIZE) * alphaStateRootCov;
                processNoiseRootCov = Eigen::MatrixXd::Identity(StateVec::COV_SIZE, StateVec::COV_SIZE) * alphaProcRootCov;
                measureNoiseRootCov = Eigen::MatrixXd::Identity(MeasureVec::COV_SIZE, MeasureVec::COV_SIZE) * alphaMeasRootCov;
            }

        };
    }
}
