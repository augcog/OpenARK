#include "stdafx.h"
#include "version.h"
#include "HandClassifier.h"

using namespace boost::filesystem;
using namespace libsvm;

namespace ark {
    namespace classifier {
        // HandClassifier implementation
        bool HandClassifier::isTrained() {
            return trained;
        }

        // SVMHandClassifier implementation

        const double SVMHandClassifier::DEFAULT_HYPERPARAMS[5 * NUM_SVMS] = {
            // gamma       coef0       C       eps     p
               2.4603,     0.5000,     0.3603, 9e-16,  0.9800,
               2.2603,     0.5000,     0.4603, 9e-16,  0.9800,
               0.8904,     0.5000,     0.1581, 1e-16,  0.9877,
               0.8219,     0.5000,     0.2945, 1e-16,  0.9963,
               0.8219,     0.5000,     0.4100, 1e-16,  0.9963,
        };

        static inline svm_parameter * makeSVMParam(const double hyperparams[5 * SVMHandClassifier::NUM_SVMS],
            int id = 0) {

            svm_parameter * param = new svm_parameter;

            // default values
            param->svm_type = EPSILON_SVR;
            param->kernel_type = RBF;
            param->degree = 5;

            param->gamma = hyperparams[id * 5];
            param->coef0 = hyperparams[id * 5 + 1];
            param->C = hyperparams[id * 5 + 2];
            param->eps = hyperparams[id * 5 + 3];
            param->p = hyperparams[id * 5 + 4];

            param->cache_size = 1000000;
            param->nu = 0.9;
            param->shrinking = 1;
            param->probability = 0;
            param->nr_weight = 0;
            param->weight_label = NULL;
            param->weight = NULL;

            return param;
        }

        void SVMHandClassifier::initSVMs(const double hyperparams[5 * NUM_SVMS]) {
            for (int i = 0; i < NUM_SVMS; ++i) {
                model[i] = nullptr;
                xstore[i] = nullptr;
                prob[i].x = nullptr;
                prob[i].y = nullptr;
                param[i] = makeSVMParam(hyperparams, i);
            }
        }

        SVMHandClassifier::SVMHandClassifier(const double hyperparams[5 * NUM_SVMS]) {
            initSVMs(hyperparams);
        }

        SVMHandClassifier::SVMHandClassifier(std::string path) {
            initSVMs();
            if (!loadFile(path)) {
                std::cerr << "WARNING: SVM initialization from path: " << path << " failed.\n";
            }
        }

        SVMHandClassifier::~SVMHandClassifier() {
            for (int i = 0; i < NUM_SVMS; ++i) {
                if (trained && model[i] != nullptr) svm_free_and_destroy_model(&model[i]);
                if (prob[i].y != nullptr) delete[] prob[i].y;
                if (xstore[i] != nullptr) delete[] xstore[i];
                if (prob[i].x != nullptr) delete[] prob[i].x;
                if (param[i] != nullptr) {
                    svm_destroy_param(param[i]);
                    delete param[i];
                }
            }
        }

        bool SVMHandClassifier::loadFile(std::string ipath) {
            path filePath(ipath);

            for (int i = 0; i < NUM_SVMS; ++i) {
                path loadPath = filePath / (std::to_string(i) + ".svmmodel");
                model[i] = svm_load_model(loadPath.string().c_str());
                if (!(trained = (model != nullptr))) break;
            }

            return trained;
        }

        bool SVMHandClassifier::exportFile(std::string opath) {
            boost::filesystem::path filePath(opath);
            for (int i = 0; i < NUM_SVMS; ++i) {
                path savePath = filePath / (std::to_string(i) + ".svmmodel");
                if (svm_save_model(savePath.string().c_str(), model[i]) != 0)
                    return false;
            }
            return true;
        }

        bool SVMHandClassifier::train(std::string dataPath) {
            if (!trained) {
                if (dataPath[dataPath.size() - 1] != '/' && dataPath[dataPath.size() - 1] != '\\') {
                    dataPath += path::preferred_separator;
                }

                std::string labelsPath = dataPath + DATA_LABELS_FILE_NAME,
                    featuresPath = dataPath + DATA_FEATURES_FILE_NAME;

                std::ifstream ifsLabels(labelsPath), ifsFeats(featuresPath);

                // ignore first line of features file (feature names); we don't need it
                std::string _; getline(ifsFeats, _);

                std::vector<int> labels, svmId;
                std::vector<std::vector<double> > feats;

                int N, numForSVM[NUM_SVMS], featsForSVM[NUM_SVMS], svmCt[NUM_SVMS];

                memset(numForSVM, 0, sizeof numForSVM);
                memset(svmCt, 0, sizeof svmCt);

                // read in total number of cases
                ifsLabels >> N;

                labels.reserve(N); feats.resize(N); svmId.reserve(N);

                int i;
                for (i = 0; i < N; ++i) {
                    std::string lb_case_name = "", ft_case_name = "";
                    int label, nFeats, nFingers;

                    if (!(ifsLabels >> lb_case_name >> label) || !(ifsFeats >> ft_case_name >> nFeats >> nFingers)) {
                        break;
                    }

                    while (lb_case_name != ft_case_name && (ifsLabels >> lb_case_name >> label)) {}

                    labels.push_back(label);

                    feats[i].clear();
                    feats[i].reserve(nFeats);
                    feats[i].push_back(nFingers);

                    for (int j = 1; j < nFeats; ++j) {
                        double fd; ifsFeats >> fd;
                        if (fd == -1 || j >= MAX_FEATURES) continue;
                        feats[i].push_back(fd);
                    }

                    int currSVMId = getSVMIdx(feats[i]);
                    svmId.push_back(currSVMId);
                    ++numForSVM[currSVMId];

                    featsForSVM[currSVMId] = (int)feats[i].size();
                }

                ifsLabels.close(); ifsFeats.close();

                // clean up old pointers & allocate memory
                for (int i = 0; i < NUM_SVMS; ++i) {
                    if (prob[i].y != nullptr) delete[] prob[i].y;
                    if (xstore[i] != nullptr) delete[] xstore[i];
                    if (prob[i].x != nullptr) delete[] prob[i].x;

                    prob[i].l = numForSVM[i];

                    std::cout << "Loaded " << numForSVM[i] << " training samples for SVM #" << i <<
                        " (" << featsForSVM[i] << " features)" << "\n";

                    // y values (labels) 
                    prob[i].y = new double[numForSVM[i]];

                    // x values (features)
                    xstore[i] = new svm_node[(featsForSVM[i] + 1) * numForSVM[i]];
                    prob[i].x = new svm_node *[numForSVM[i]];
                }


                for (i = 0; i < (int)svmId.size(); ++i) {
                    int id = svmId[i];

                    // set label, etc.
                    prob[id].y[svmCt[id]] = labels[i] * 2 - 1;
                    prob[id].x[svmCt[id]] = xstore[id] + (featsForSVM[id] + 1) * svmCt[id];

                    for (int j = 0; j < featsForSVM[id] - 1; ++j) {
                        prob[id].x[svmCt[id]][j].index = j + 1;
                        prob[id].x[svmCt[id]][j].value = feats[i][j + 1];
                    }

                    prob[id].x[svmCt[id]][featsForSVM[id]].index = -1;

                    ++svmCt[id];
                }

                for (int i = 0; i < NUM_SVMS; ++i) {
                    std::cout << "Training SVM " << i << "...\n";
                    model[i] = svm_train(&prob[i], param[i]);
                }

                trained = true;

                std::cout << "\nTesting...\n";

                int good = 0, goodSVM[NUM_SVMS];
                memset(goodSVM, 0, sizeof goodSVM);

                for (i = 0; i < (int)svmId.size(); ++i) {
                    double res = classify(feats[i]);
                    if (res < 0.5 && labels[i] == 0 || res > 0.5 && labels[i] == 1) {
                        ++good;
                        ++goodSVM[svmId[i]];
                    }
                }

                std::cout << "Training Results:\n";
                for (int i = 0; i < NUM_SVMS; ++i) {
                    std::cout << "\tSVM " << i << ":" << (double)goodSVM[i] / numForSVM[i] * 100 << "% Correct\n";
                }

                std::cout << "Overall:" << (double)good / (svmId.size()) * 100 << "% Correct\n\n";
            }

            return trained;
        }

        double SVMHandClassifier::classify(const std::vector<double> features) {
            if (!trained) throw ClassifierNotTrainedException();

            //int nFeats = features::getNumFeatures();

            // if no fingers, predict not hand
            if (features.size() == 0) return 0.0;

            int nFeat = (int)features.size();

            svm_node * nd = new svm_node[nFeat + 2];
            int num_fingers = (int)features[0];

            if (nFeat > MAX_FEATURES) nFeat = MAX_FEATURES;

            for (int i = 1; i < nFeat; ++i) {
                nd[i - 1].index = i;
                nd[i - 1].value = features[i];
            }

            nd[nFeat - 1].index = -1;

            int svmIdx = getSVMIdx(features);

            double result = (svm_predict(model[svmIdx], nd) + 1.0) / 2.0;
            delete[] nd;

            // normalize confidences for each SVM
            if (svmIdx <= 1)
                return (std::max(0.44, std::min(0.54, result)) - 0.44) * 10.0;
            else //if (svmIdx > 1)
                return (std::max(0.45, std::min(0.55, result)) - 0.45) * 10.0;
        }

        inline int SVMHandClassifier::getSVMIdx(const std::vector<double> features) {
            int numFingers = (int)features[0];
            return std::min(numFingers, NUM_SVMS - 1);
        }
    }
}
