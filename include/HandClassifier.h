#pragma once

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <string>
#include <vector>
#include <exception>

#include "Version.h"
#include "Hand.h"

namespace ark {
    namespace classifier {

        class ClassifierNotTrainedException : public std::exception {};
        class InvalidFeatureVectorException : public std::exception {};

        /**
        * Name of file containing the data labels (path from data dir)
        */
        const std::string DATA_LABELS_FILE_NAME = "labels.txt";

        /**
        * Name of file containing the feature information (path from data dir)
        */
        const std::string DATA_FEATURES_FILE_NAME = "handfeatures.csv";

        /**
         * Abstract base class for hand and finger classifiers
         */
        class Classifier {
        public:
            /**
             * Import the classifier model from a file
             * @param path path to import from
             * @return true on success, false on error.
             */
            virtual bool loadFile(std::string path) = 0;

            /**
             * Export the classifier model to a file.
             * @param path path to export to
             * @return true if success, false on error.
             */
            virtual bool exportFile(std::string path) const = 0;

            /**
             * Start training this classifier with data from the specified path.
             * @param dataPath path to the training data directory
             * @param hyperparams hyperparameter array
             * @return true on success, false on error. If already trained, returns true.
             */
            virtual bool train(std::string dataPath, const double hyperparams[]) = 0;

            /**
             * Returns true if this classifier has finished training
             * @return true if classifier is trained
             */
            virtual bool isTrained() const;

            /**
             * Use this classifier to classify a features vector representing an object.
             * If the classifier has yet to be trained, throws ClassifierNotTrainedException.
             * @param [in] vector of features (CV_32F; 1xN)
             * @return double value indicating category the object belongs to
             */
            virtual float classify(const cv::Mat & features) const = 0;

        protected:
            /**
            * True if the classifer has been trained.
            */
            bool trained = false;

            /** Helper for computing mean and variance */
            static void computeMeanAndVariance(const std::vector<ark::Vec3f> & points, cv::Vec3f center,
                double & avg_dist, double & var_dist, double & avg_depth, double & var_depth);
        };

        /**
         * (OpenCV) SVM-based hand classifier
         */
        class SVMHandClassifier : public Classifier {
        public:
            /**
             * Number of SVMs used
             * 1 SVM is used for each number of visible fingers, i.e. 1 for hands with 1 visible finger,
             * 1 for hands with 2 visible fingers, etc.
             */
            static const int NUM_SVMS = 4;

            /**
            * Default SVM hyperparameters
            */
            static const double DEFAULT_HYPERPARAMS[5 * NUM_SVMS];

            /**
             * Create a new, untrained SVM hand classifier
             */
            SVMHandClassifier();

            /** Construct a SVM hand classifier by
             *  loading the classifier models from disk.
             *  @param path path to directory with model files
             */
            explicit SVMHandClassifier(const char * path);

            /** Construct a SVM hand classifier by
             *  loading the classifier models from disk. Attempts multiple locations.
             *  @param paths paths to attempt to load the model files from
             */
            explicit SVMHandClassifier(const char * paths[]);

            /**
             * Destroy this SVM hand classifier
             */
            ~SVMHandClassifier();

            /**
             * Load the SVM models from disk
             * @param path directory to load models from.
             * @return true on success, false on error.
             */
            bool loadFile(std::string path) override;

            /**
             * Write the SVM models to disk.
             * @param path directory to export models to.
             * @return true if success, false on error.
             */
            bool exportFile(std::string path) const override;

            /**
             * Start training this classifier with data from the specified path.
             * @param dataPath path to the training data directory
             * @param hyperparams hyperparameter array
             * @return true on success, false on error. If already trained, returns true.
             */
            virtual bool train(std::string dataPath,
                const double hyperparams[5 * NUM_SVMS] = DEFAULT_HYPERPARAMS) override;

            /**
             *  Use this classifier to classify a feature vector representing a hand object.
             *  Returns a double between 0 and 1.
             * '1' indicates that we are fully confident that the object is a hand, and vice versa.
             *  If the classifier has yet to be trained, throws ClassifierNotTrainedException.
             *  @param [in] features vector of features (index 0 should be number of fingers) (CV_32F; 1 x N)
             *  @return Confidence that object is a hand (double value between 0 and 1)
             */
            float classify(const cv::Mat & features) const override;

            /**
             *  Use this classifier to classify a Hand instance. Extracts features from this hand automatically.
             * '1' indicates that we are fully confident that the object is a hand, and vice versa.
             *  If the classifier has yet to be trained, throws ClassifierNotTrainedException.
             *  @param [in] hand Hand instance
             *  @param [in] depth_map depth map (note: must be CV_32FC3)
             *  @param top_left optionally, top left point represented in depth map (x, y coordinates to translate by)
             *  @param full_wid optionally, size of full depth map. By default, uses width of depth_map
             *  @return Confidence that object is a hand (double value between 0 and 1)
             */
            float classify(ark::Hand & hand,
                const cv::Mat & depth_map, cv::Point top_left = cv::Point(0, 0), int full_wid = -1) const;

            /**
            * Get the index of the SVM this classifier would use for a certain feature vector.
            * Useful for checking which SVM is working well and which is not
            * Note: 1 SVM is used for each number of visible fingers, i.e. 1 for hands with 1 visible finger,
            * 1 for hands with 2 visible fingers, etc.
            * @param features vector of features (index 0 should be number of fingers)
            * @return index of SVM used
            */
            static int getSVMIdx(const cv::Mat & features);

            /**
            * Get the index of the SVM this classifier would use for the given number of fingers
            * Useful for checking which SVM is working well and which is not
            * Note: 1 SVM is used for each number of visible fingers, i.e. 1 for hands with 1 visible finger,
            * 1 for hands with 2 visible fingers, etc.
            * @param features vector of features (index 0 should be number of fingers)
            * @return index of SVM used
            */
            static int getSVMIdx(int num_fingers);

            /** Extract hand-specific features from a given Hand object and depth map
             *  @param [in] hand Hand instance
             *  @param [in] depth_map depth map (note: must be CV_32FC3)
             *  @param top_left optionally, top left point represented in depth map (x, y coordinates to translate by)
             *  @param full_wid optionally, size of full depth map. By default, uses width of depth_map
             *  @return vector of features (type CV_32F, 1xN)
             */
            static cv::Mat extractFeatures(ark::Hand & hand, const cv::Mat & depth_map,
                cv::Point top_left = cv::Point(0, 0), int full_wid = -1);

        private:
            /**
             * Maximum number of features. Any additonal features will be cut off.
             */
            static const int MAX_FEATURES = 57;

            // SVM storage
            cv::Ptr<cv::ml::SVM> svm[NUM_SVMS];

            /**
            * Helper for initializing classifiers
            */
            void initSVMs(const double hyperparams[5 * NUM_SVMS] = DEFAULT_HYPERPARAMS);
        };

        /**
         * SVM-based hand validator
         */
        class SVMHandValidator : public Classifier {
        public:
            /**
            * Default SVM hyperparameters
            */
            static const double DEFAULT_HYPERPARAMS[5];

            /**
             * Create a new, untrained SVM hand classifier
             */
            SVMHandValidator();

            /** Construct a SVM hand classifier by
             *  loading the classifier models from disk.
             *  @param path path to directory with model files
             */
            explicit SVMHandValidator(const char * path);

            /** Construct a SVM hand classifier by
             *  loading the classifier models from disk. Attempts multiple locations.
             *  @param paths paths to attempt to load the model files from
             */
            explicit SVMHandValidator(const char * paths[]);

            /**
             * Destroy this SVM hand classifier
             */
            ~SVMHandValidator();

            /**
             * Load the SVM models from disk
             * @param path directory to load models from.
             * @return true on success, false on error.
             */
            bool loadFile(std::string path) override;

            /**
             * Write the SVM models to disk.
             * @param path directory to export models to.
             * @return true if success, false on error.
             */
            bool exportFile(std::string path) const override;

            /**
             * Start training this classifier with data from the specified path.
             * @param dataPath path to the training data directory
             * @param hyperparams hyperparameter array
             * @return true on success, false on error. If already trained, returns true.
             */
            virtual bool train(std::string dataPath,
                const double hyperparams[5] = DEFAULT_HYPERPARAMS) override;

            /**
             *  Use this classifier to validate a feature vector representing a hand object.
             *  Returns a double between 0 and 1.
             * '1' indicates that we are fully confident that the object is a hand, and vice versa.
             *  If the classifier has yet to be trained, throws ClassifierNotTrainedException.
             *  @param [in] features vector of features (CV_32F; 1 x N)
             *  @return Confidence that object is a hand (double value between 0 and 1)
             */
            float classify(const cv::Mat & features) const override;

            /**
             *  Use this classifier to validate a Hand instance. Extracts features from this hand automatically.
             * '1' indicates that we are fully confident that the object is a hand, and vice versa.
             *  If the classifier has yet to be trained, throws ClassifierNotTrainedException.
             *  @param [in] hand Hand instance
             *  @param [in] depth_map depth map (note: must be CV_32FC3)
             *  @param top_left optionally, top left point represented in depth map (x, y coordinates to translate by)
             *  @param full_wid optionally, size of full depth map. By default, uses width of depth_map
             *  @param num_features number of circle features to extract
             *  @param avg_size size of square around each point on XYZ map 
             *                  to average when obtaining XYZ coordinates from IJ coordinates
             *  @return Confidence that object is a hand (double value between 0 and 1)
             */
            float classify(ark::Hand & hand,
                const cv::Mat & depth_map, cv::Point top_left = cv::Point(0, 0), int full_wid = -1, 
                int num_features = 48, int avg_size = 5) const;

            /** Extract features from a given Hand instance and depth map for use with SVMHandValidator
             *  @param [in] hand Hand instance
             *  @param [in] depth_map depth map (note: must be CV_32FC3)
             *  @param top_left optionally, top left point represented in depth map (x, y coordinates to translate by)
             *  @param full_wid optionally, size of full depth map. By default, uses width of depth_map
             *  @param num_features number of features to extract
             *  @param avg_size size of square around each point on XYZ map 
             *                  to average when obtaining XYZ coordinates from IJ coordinates
             *  @return vector of features (type CV_32F, 1xN)
             */
            static cv::Mat extractFeatures(ark::Hand & hand,
                const cv::Mat & depth_map, cv::Point top_left = cv::Point(0, 0), int full_wid = -1,
                int num_features = 48, int avg_size = 5);

        private:

            // the SVM
            cv::Ptr<cv::ml::SVM> svm;

            /**
            * Helper for initializing classifiers
            */
            void initSVMs(const double hyperparams[5] = DEFAULT_HYPERPARAMS);
        };
    }
}
