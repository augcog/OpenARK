#include "stdafx.h"
#include "version.h"
#include "Hand.h"
#include "Util.h"
#include "HandClassifier.h"

namespace ark {
    namespace classifier {
        // Classifier implementation
        bool Classifier::isTrained() const {
            return trained;
        }

        void Classifier::computeMeanAndVariance(const std::vector<ark::Vec3f> & points,
            cv::Vec3f center, double& avg_dist, double& var_dist, double& avg_depth, double& var_depth) {

            avg_dist = avg_depth = 0;
            int totalpts = 0;

            for (const ark::Vec3f pt : points) {
                avg_dist +=
                    sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) +
                    (pt[1] - center[1]) * (pt[1] - center[1]));
                avg_depth += pt[2];
                ++totalpts;
            }

            if (totalpts == 0) {
                avg_dist = avg_depth = 1.0;
                var_dist = var_depth = 0.0;
                return;
            }

            avg_dist /= totalpts;
            avg_depth /= totalpts;

            var_dist = var_depth = 0;

            for (const ark::Vec3f pt : points) {
                double dist =
                    sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) +
                    (pt[1] - center[1]) * (pt[1] - center[1]));
                var_dist += (dist - avg_dist) * (dist - avg_dist);
                var_depth += (pt[2] - avg_depth) * (pt[2] - avg_depth);
            }

            var_dist /= totalpts;
            var_depth /= totalpts;
        }

        // SVMHandClassifier implementation

        const double SVMHandClassifier::DEFAULT_HYPERPARAMS[5 * NUM_SVMS] = {
            // gamma       coef0       C       eps     p
            0.8219,     0.5000,     0.5000, 9e-16,  0.0548,
            0.3425,     0.5000,     0.4041, 1e-16,  0.0548,
            0.3425,     0.5000,     0.5493, 1e-16,  0.0548,
            0.2740,     0.5000,     0.4100, 1e-16,  0.0548,
        };

        void SVMHandClassifier::initSVMs(const double hyperparams[5 * NUM_SVMS]) {
            for (int i = 0; i < NUM_SVMS; ++i) {
                svm[i] = cv::ml::SVM::create();
                svm[i]->setType(cv::ml::SVM::EPS_SVR);
                svm[i]->setKernel(cv::ml::SVM::RBF);
                svm[i]->setGamma(hyperparams[i * 5 + 0]);
                svm[i]->setCoef0(hyperparams[i * 5 + 1]);
                svm[i]->setC(hyperparams[i * 5 + 2]);
                svm[i]->setP(hyperparams[i * 5 + 4]);
            }
        }

        SVMHandClassifier::SVMHandClassifier() { }

        SVMHandClassifier::SVMHandClassifier(const char * path) {
            initSVMs();
            loadFile(path);
        }

        SVMHandClassifier::SVMHandClassifier(const char * paths[]) {
            initSVMs();

            for (int i = 0; ; ++i) {
                if (strcmp(paths[i], "\n") == 0) {
                    break;
                }
                if (loadFile(paths[i])) {
                    break;
                }
            }
        }

        SVMHandClassifier::~SVMHandClassifier() {
            // do nothing

        }

        bool SVMHandClassifier::loadFile(std::string ipath) {
            using namespace boost::filesystem;

            const char * env = std::getenv("OPENARK_DIR");
            path filePath(ipath);

            if (env) filePath = path(env) / filePath;

            trained = true;

            for (int i = 0; i < NUM_SVMS; ++i) {
                path loadPath = filePath / ("svm_" + std::to_string(i) + ".xml");

                if (!boost::filesystem::exists(loadPath)){
                    trained = false;
                    break;
                }

                svm[i] = cv::ml::SVM::load(loadPath.string());
                if (!svm[i]->isTrained()) {
                    trained = false;
                    break;
                }
            }

            return trained;
        }

        bool SVMHandClassifier::exportFile(std::string opath) const {
            boost::filesystem::path filePath(opath);
            for (int i = 0; i < NUM_SVMS; ++i) {
                boost::filesystem::path savePath = filePath / ("svm_" + std::to_string(i) + ".xml");
                svm[i]->save(savePath.string());
            }
            return true;
        }

        bool SVMHandClassifier::train(std::string dataPath, const double hyperparams[5 * NUM_SVMS]) {
            initSVMs(hyperparams);

            if (dataPath[dataPath.size() - 1] != '/' && dataPath[dataPath.size() - 1] != '\\') {
                dataPath += boost::filesystem::path::preferred_separator;
            }

            std::string labelsPath = dataPath + DATA_LABELS_FILE_NAME,
                featuresPath = dataPath + DATA_FEATURES_FILE_NAME;

            std::ifstream ifsLabels(labelsPath), ifsFeats(featuresPath);

            // total cases
            int N; ifsLabels >> N;

            // ignore first line of features file (feature names); we don't need it
            std::string _; getline(ifsFeats, _);

            // record number of features for SVM #
            int numFeats[NUM_SVMS];

            // record number of fingers for SVM #
            int numFing[NUM_SVMS];

            // record number of samples for SVM #
            int numSamples[NUM_SVMS];

            memset(numFeats, 0x3f, sizeof numFeats); // large number
            memset(numFing, 0x3f, sizeof numFing); // large number
            memset(numSamples, 0, sizeof numSamples);

            // pre-scan features file to determine data matrix dimensions
            for (int i = 0; i < N; ++i) {
                std::string name;
                int numFeatures, numFingers;
                ifsFeats >> name >> numFeatures >> numFingers;

                int svmID = getSVMIdx(numFingers);

                if (svmID >= 0) {
                    ++numSamples[svmID];
                    numFeats[svmID] = std::min(numFeatures, numFeats[svmID]);
                    numFing[svmID] = std::min(numFingers, numFing[svmID]);
                }

                // ignore rest of line
                std::getline(ifsFeats, _);
            }

            // start from beginning
            ifsFeats.seekg(0, ios::beg); getline(ifsFeats, _);

            cv::Mat data[NUM_SVMS], labels[NUM_SVMS];

            for (int i = 0; i < NUM_SVMS; ++i) {
                data[i].create(numSamples[i], numFeats[i] - 1, CV_32F);
                labels[i].create(1, numSamples[i], CV_32S);
                std::cerr << numFeats[i] << " ";
            }

            // use to record samples read
            memset(numSamples, 0, sizeof numSamples);

            int i;
            for (i = 0; i < N; ++i) {
                std::string lbName = "", ftName = "";
                int label, numFeatures, numFingers;

                // synchronize
                if (!(ifsLabels >> lbName >> label) || !(ifsFeats >> ftName >> numFeatures >> numFingers)) {
                    break;
                }
                while (lbName != ftName && (ifsLabels >> lbName >> label)) {}

                int currSVMId = getSVMIdx(numFingers);
                if (currSVMId < 0) {
                    std::getline(ifsFeats, _);
                    continue;
                }

                // add label
                labels[currSVMId].at<int>(0, numSamples[currSVMId]) = label;
                float * ptr = data[currSVMId].ptr<float>(numSamples[currSVMId]++);

                // read features
                for (int j = 0; j < numFeatures - 1; ++j) {
                    if (j >= numFeats[currSVMId] - 1) {
                        // ignore
                        std::getline(ifsFeats, _);
                        break;
                    }
                    ifsFeats >> ptr[j];
                }
            }

            // clean up old pointers & allocate memory
            for (int i = 0; i < NUM_SVMS; ++i) {
                std::cout << "Loaded " << numSamples[i] << " training samples for SVM #" << i <<
                    " (" << numSamples[i] << " features)" << "\n";
            }

            for (int i = 0; i < NUM_SVMS; ++i) {
                std::cout << "Training SVM " << i << "...\n";
                auto trainData = cv::ml::TrainData::create(data[i], cv::ml::ROW_SAMPLE, labels[i]);
                svm[i]->train(trainData);
                trainData.release();
            }

            trained = true;

            std::cout << "\nTesting...\n";

            int good = 0, goodSVM[NUM_SVMS];
            memset(goodSVM, 0, sizeof goodSVM);

            ifsLabels.close(); ifsFeats.close();

            for (i = 0; i < NUM_SVMS; ++i) {
                for (int j = 0; j < data[i].rows; ++j) {
                    cv::Mat feats(1, numFeats[i], CV_32F);
                    feats.at<float>(0, 0) = numFing[i];

                    int label = labels[i].at<int>(0, j);

                    float * ptr = data[i].ptr<float>(j);

                    for (int k = 0; k < data[i].cols; ++k) {
                        feats.at<float>(0, k + 1) = ptr[k];
                    }

                    double res = classify(feats);
                    if (res < 0.5 && label == 0 || res > 0.5 && label == 1) {
                        ++good;
                        ++goodSVM[i];
                    }
                }
            }

            std::cout << "Training Results:\n";
            for (int i = 0; i < NUM_SVMS; ++i) {
                std::cout << "\tSVM " << i << ":" <<
                    (double)goodSVM[i] / numSamples[i] * 100 << "% Correct\n";
            }

            std::cout << "Overall:" << (double)good / N * 100 << "% Correct\n\n";

            return trained;
        }

        float SVMHandClassifier::classify(const cv::Mat & features) const {
            if (!trained) throw ClassifierNotTrainedException();

            // if no fingers, predict not hand
            if (!features.data || features.cols == 0) return 0.0;

            int nFeat = features.cols;
            if (nFeat > MAX_FEATURES) nFeat = MAX_FEATURES;

            cv::Mat samples = features(cv::Rect(1, 0, nFeat - 1, 1));
            int svmIdx = getSVMIdx(features);
            double result = svm[svmIdx]->predict(samples);

            // range [0, 1]
            return std::max(std::min(1.0, result), 0.0);
        }

        float SVMHandClassifier::classify(ark::Hand & hand, const cv::Mat & depth_map, cv::Point top_left, int full_wid) const
        {
            cv::Mat features = extractFeatures(hand, depth_map, top_left, full_wid);
            return classify(features);
        }

        int SVMHandClassifier::getSVMIdx(const cv::Mat & features) {
            int numFingers = features.at<float>(0, 0);
            return getSVMIdx(numFingers);
        }

        int SVMHandClassifier::getSVMIdx(int num_fingers) {
            return std::min(num_fingers - 1, NUM_SVMS - 1);
        }

        cv::Mat SVMHandClassifier::extractFeatures(ark::Hand & hand,
            const cv::Mat & depth_map, cv::Point top_left, int full_wid) {

            if (full_wid < 0) full_wid = depth_map.cols;

            std::vector<float> result;

            int nFingers = hand.getNumFingers();
            result.push_back(nFingers);
            if (nFingers == 0) return cv::Mat::zeros(1, 1, CV_32F);

            cv::Vec3f center = hand.getPalmCenter();
            cv::Point centerij = hand.getPalmCenterIJ();

            double avgdist, vardist, avgdepth, vardepth;
            computeMeanAndVariance(hand.getPoints(), center, avgdist, vardist, avgdepth, vardepth);

            if (nFingers > 1) result.reserve(nFingers * 13 + 10);
            else result.reserve(20);

            double area = hand.getSurfArea();

            // average distance to palm center (of all points)
            result.push_back(avgdist * 20.0);

            // variance to palm center (of all points)
            result.push_back(sqrt(vardist) * 25.0);

            // surface area
            result.push_back(area * 10.00);

            // variance of depth (average of depth not used)
            result.push_back(sqrt(vardepth) * 25.0);

            std::vector<ark::Point2i> cont = hand.getContour(),
                hull = hand.getConvexHull();
            std::vector<ark::Vec3f> wrist = hand.getWrist();

            double contArea = cv::contourArea(cont);

            // contour area as fraction of hull area
            result.push_back(contArea / cv::contourArea(hull));

            cv::Rect bounds = hand.getBoundingBox();

            // contour area as fraction of bounding box area
            result.push_back(contArea / (bounds.width * bounds.height));

            // arc length of contour as fraction of arc length of hull
            result.push_back(cv::arcLength(cont, true) /
                cv::arcLength(hull, true) * 0.5);

            int pa, pb;
            double diam = util::diameter(cont, pa, pb);

            // inscribed circle radius as fraction of diameter
            result.push_back(hand.getCircleRadius() / diam * 2.0);

            ark::Vec3f paXYZ = ark::util::averageAroundPoint(depth_map, cont[pa]);
            ark::Vec3f pbXYZ = ark::util::averageAroundPoint(depth_map, cont[pb]);

            // diameter of cluster, projected to 3D
            result.push_back(ark::util::euclideanDistance(paXYZ, pbXYZ));

            // wrist width
            result.push_back(ark::util::euclideanDistance(wrist[0], wrist[1]));

            typedef std::pair<boost::polygon::detail::fpt64, int> pfi;

            // (order the fingers by length)
            std::vector<pfi> fingerOrder;

            ark::Vec3f midWrist = wrist[0] + (wrist[1] - wrist[0]) / 2;

            double avgLen = 0.0;
            double avgMidWristDist = 0.0;
            for (int i = 0; i < nFingers; ++i) {
                cv::Vec3f finger = hand.getFingers()[i], defect = hand.getDefects()[i];

                double finger_len = ark::util::euclideanDistance(finger, defect);
                avgLen += finger_len;
                avgMidWristDist += ark::util::euclideanDistance(finger, midWrist);

                fingerOrder.push_back(pfi(finger_len, i));
            }
            std::sort(fingerOrder.begin(), fingerOrder.end(), std::greater<pfi>());

            // average finger length
            result.push_back(avgLen / nFingers * 5.0);

            // average distance from fingers to middle of wrist
            result.push_back(avgMidWristDist / nFingers * 2.0);

            auto fingers = hand.getFingers(), defects = hand.getDefects();
            auto fingersIJ = hand.getFingersIJ(), defectsIJ = hand.getDefectsIJ();

            for (int k = 0; k < nFingers; ++k) {
                int j = fingerOrder[k].second;

                const cv::Vec3f & finger = fingers[j], &defect = defects[j];
                const cv::Point & fingerij = fingersIJ[j], &defectij = defectsIJ[j];

                result.push_back(ark::util::euclideanDistance(finger, defect) * 5.0);
                result.push_back(ark::util::euclideanDistance(defect, center) * 5.0);
                result.push_back(ark::util::euclideanDistance(finger, center) * 5.0);

                result.push_back(ark::util::angleBetween3DVec(finger, defect, center) / PI);
                result.push_back(ark::util::angleBetweenPoints(fingerij, centerij, defectij) / PI);

                result.push_back(ark::util::pointToAngle(fingerij - centerij));
                result.push_back(ark::util::pointToAngle(defectij - centerij));

                double minDistDefect = depth_map.cols, minDistFinger = depth_map.cols;
                double maxDistDefect = 0, maxDistFinger = 0;

                if (nFingers > 1) {
                    for (int jj = 0; jj < nFingers; ++jj) {
                        if (j == jj) continue;

                        double distDefect = ark::util::euclideanDistance(defect, defects[jj]),
                            distFinger = ark::util::euclideanDistance(finger, fingers[jj]);

                        if (distDefect < minDistDefect) minDistDefect = distDefect;
                        if (distDefect > maxDistDefect) maxDistDefect = distDefect;
                        if (distFinger < minDistFinger) minDistFinger = distFinger;
                        if (distFinger > maxDistFinger) maxDistFinger = distFinger;
                    }

                    result.push_back(minDistFinger * 5.0);
                    result.push_back(maxDistFinger * 5.0);
                    result.push_back(minDistDefect * 5.0);
                    result.push_back(maxDistDefect * 5.0);
                }
            }

            for (unsigned i = 0; i < result.size(); ++i) {
                if (isnan(result[i])) {
                    result[i] = 1.0;
                }
                else if (result[i] >= FLT_MAX) {
                    result[i] = 100.0;
                }
            }

            cv::Mat mat(1, (int)result.size(), CV_32F);
            for (uint i = 0; i < result.size(); ++i) {
                mat.at<float>(0, i) = result[i];
            }
            return mat;
        }

        // SVMHandValidator implementation

        const double SVMHandValidator::DEFAULT_HYPERPARAMS[5] = {
            // gamma       coef0       C       eps     p
            1.5068,     0.5000,     0.6301, 1.5e-16,  0.0548,
        };

        void SVMHandValidator::initSVMs(const double hyperparams[5]) {
            svm = cv::ml::SVM::create();
            svm->setType(cv::ml::SVM::EPS_SVR);
            svm->setKernel(cv::ml::SVM::RBF);
            svm->setGamma(hyperparams[0]);
            svm->setCoef0(hyperparams[1]);
            svm->setC(hyperparams[2]);
            svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::EPS, 10000, hyperparams[4]));
            svm->setP(hyperparams[4]);
        }

        SVMHandValidator::SVMHandValidator() { }

        SVMHandValidator::SVMHandValidator(const char * path) {
            initSVMs();
            loadFile(path);
        }

        SVMHandValidator::SVMHandValidator(const char * paths[]) {
            initSVMs();

            for (int i = 0; ; ++i) {
                if (strcmp(paths[i], "\n") == 0) {
                    break;
                }
                if (loadFile(paths[i])) {
                    break;
                }
            }
        }

        SVMHandValidator::~SVMHandValidator() {
            // do nothing

        }

        bool SVMHandValidator::loadFile(std::string ipath) {
            using namespace boost::filesystem;

            const char * FILE_NAME = "svm.xml";
            const char * ENV_VAR_NAME = "OPENARK_DIR";

            path loadPath(ipath / FILE_NAME);
            if (!boost::filesystem::exists(loadPath)) {
                const char * env = std::getenv(ENV_VAR_NAME);
                if (env) {
                    loadPath = path(env) / ipath / FILE_NAME;
                }

                if (!env || !boost::filesystem::exists(loadPath)) {
                    trained = false;
                    return trained;
                }
            }

            svm = cv::ml::SVM::load(loadPath.string());
            trained = true;
            return trained;
        }

        bool SVMHandValidator::exportFile(std::string opath) const {
            boost::filesystem::path filePath(opath);
            boost::filesystem::path savePath = filePath / "svm.xml";
            svm->save(savePath.string());
            return true;
        }

        bool SVMHandValidator::train(std::string dataPath, const double hyperparams[5]) {
            initSVMs(hyperparams);

            if (dataPath[dataPath.size() - 1] != '/' && dataPath[dataPath.size() - 1] != '\\') {
                dataPath += boost::filesystem::path::preferred_separator;
            }

            std::string labelsPath = dataPath + DATA_LABELS_FILE_NAME,
                featuresPath = dataPath + DATA_FEATURES_FILE_NAME;

            std::ifstream ifsLabels(labelsPath), ifsFeats(featuresPath);

            // total cases
            int N; ifsLabels >> N;

            // ignore first line of features file (feature names); we don't need it
            std::string _; getline(ifsFeats, _);

            int numFeats;
            ifsFeats >> _ >> numFeats;

            // start from beginning
            ifsFeats.seekg(0, ios::beg); getline(ifsFeats, _);

            cv::Mat data, labels;
            data.create(N, numFeats, CV_32F);
            labels.create(1, N, CV_32S);

            int i;
            for (i = 0; i < N; ++i) {
                std::string lbName = "", ftName = "";
                int label, numFeatures;

                // synchronize
                if (!(ifsLabels >> lbName >> label) || !(ifsFeats >> ftName >> numFeatures)) {
                    break;
                }
                while (lbName != ftName && (ifsLabels >> lbName >> label)) {}

                // add label
                labels.at<int>(0, i) = label;
                float * ptr = data.ptr<float>(i);

                // read features
                for (int j = 0; j < numFeatures; ++j) {
                    if (j >= numFeats) {
                        // ignore
                        std::getline(ifsFeats, _);
                        break;
                    }
                    ifsFeats >> ptr[j];
                }
            }

            // clean up old pointers & allocate memory
            std::cout << "Loaded " << i <<
                " training samples (" << numFeats << " features)" << "\n";

            std::cout << "Training SVM...\n";
            cv::Ptr<cv::ml::TrainData> trainData =
                cv::ml::TrainData::create(data, cv::ml::ROW_SAMPLE, labels);
            svm->train(trainData);
            trainData.release();

            trained = true;

            std::cout << "\nTesting...\n";

            int good = 0;

            ifsLabels.close(); ifsFeats.close();

            for (int j = 0; j < data.rows; ++j) {
                cv::Mat feats = data(cv::Rect(0, j, data.cols, 1));
                int label = labels.at<int>(0, j);

                double res = classify(feats);
                if (res < 0.5 && label == 0 || res > 0.5 && label == 1) {
                    ++good;
                }
            }

            std::cout << "Training Results:\n";
            std::cout << (double)good / N * 100.0 << "% Correct\n\n";

            return trained;
        }

        float SVMHandValidator::classify(const cv::Mat & features) const {
            if (!trained) throw ClassifierNotTrainedException();

            // if no fingers, predict not hand
            if (features.data == nullptr || features.cols == 0) return 0.0f;
            float result = svm->predict(features);

            // range [0, 1]
            return std::max(std::min(1.0f, result), 0.0f);
        }

        float SVMHandValidator::classify(ark::Hand & hand, const cv::Mat & depth_map, int num_features, int avg_size, cv::Point top_left, int full_wid) const
        {
            cv::Mat features = extractFeatures( hand, depth_map, num_features, avg_size, top_left, full_wid);
            return classify(features);
        }

        cv::Mat SVMHandValidator::extractFeatures(ark::Hand & hand, 
            const cv::Mat & depth_map, int num_features, int avg_size, cv::Point top_left, int full_wid)
        {
            Point2f center = hand.getPalmCenterIJ() - top_left;
            Vec3f centerXYZ = hand.getPalmCenter();

            double wristDir = util::pointToAngle(-hand.getDominantDirection());

            cv::Mat result(1, num_features, CV_32F);

            double step = PI / num_features * 2.0;

            float * ptr = result.ptr<float>(0);

#ifdef DEBUG
            cv::Mat visual = hand.getDepthMap().clone();
#endif

            double angle = 0.0;
            for (int i = 0; i < num_features; ++i) {
                 float rad = util::radiusInDirection(depth_map, center, angle, wristDir);
                 if (rad < 0.0f) rad = 0.0f;

                 Point2f dir = util::angleToPoint(angle + wristDir);
                 Point2i farPoint = center + rad * dir;

                 Vec3f pos = util::averageAroundPoint(depth_map, farPoint, avg_size);
                 ptr[i] = util::euclideanDistance(pos, centerXYZ) * 10.0;

#ifdef DEBUG
                 cv::circle(visual, farPoint + top_left, 5, cv::Scalar(255, 0, 255), 2);
#endif

                 angle += step;
            }

#ifdef DEBUG
                 cv::circle(visual, Point2i(center) + top_left, 5, cv::Scalar(0, 255, 100), 2);
                 cv::imshow("[SVM Feature Extraction]", visual);
#endif

            return result;
        }


    }
}
