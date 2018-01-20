#pragma once

#include "stdafx.h"
#include "version.h"

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
         * Abstract base class for hand classifiers
         */
        class HandClassifier {
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
            virtual bool exportFile(std::string path) = 0;

            /**
             * Start training this classifier with data from the specified path.
             * @param dataPath path to the training data directory
             * @return true on success, false on error. If already trained, returns true.
             */
            virtual bool train(std::string dataPath) = 0;

            /**
             * Returns true if this classifier has finished training
             * @return true if classifier is trained
             */
            virtual bool isTrained();

            /**
             * Use this classifier to classify a features vector representing an object.
             * If the classifier has yet to be trained, throws ClassifierNotTrainedException.
             * @param vector of features
             * @return double value indicating category the object belongs to
             */
            virtual double classify(const std::vector<double> features) = 0;

        protected:
            /**
            * True if the classifer has been trained.
            */
            bool trained = false;
        };

        /**
         * SVM-based hand classifier
         */
        class SVMHandClassifier : public HandClassifier {
        public:
            /**
             * Number of SVMs used
             * 1 SVM is used for each number of visible fingers, i.e. 1 for hands with 1 visible finger,
             * 1 for hands with 2 visible fingers, etc.
             */
            static const int NUM_SVMS = 5;

            /**
            * Default SVM hyperparameters
            */
            static const double DEFAULT_HYPERPARAMS[5 * NUM_SVMS];

            /**
             * Create a new, untrained SVM hand classifier
             * @param hyperparams array of hyperparameters (defaults to DEFAULT_HYPERPARAMS)
             */
            SVMHandClassifier(const double hyperparams[5 * NUM_SVMS] = DEFAULT_HYPERPARAMS);

            /** Construct a SVM hand classifier by
             *  loading the classifier models from disk.
             *  @param path path to directory with model files
             */
            explicit SVMHandClassifier(std::string path);

            /**
             * Destroy this SVM hand classifier
             */
            ~SVMHandClassifier();

            /**
             * Load the SVM models from disk
             * @param path directory to load models from.
             * @return true on success, false on error.
             */
            bool loadFile(std::string path);

            /**
             * Write the SVM models to disk.
             * @param path directory to export models to.
             * @return true if success, false on error.
             */
            bool exportFile(std::string path);

            /**
             * Start training this classifier with data from the specified path.
             * @param dataPath path to the training data directory
             * @return true on success, false on error. If already trained, returns true.
             */
            virtual bool train(std::string dataPath);

            /**
             *  Use this classifier to classify a feature vector representing an object.
             *  Returns a double between 0 and 1.
             * '1' indicates that we are fully confident that the object is a hand, and vice versa.
             *  If the classifier has yet to be trained, throws ClassifierNotTrainedException.
             *  @param features vector of features (index 0 should be number of fingers)
             *  @return Confidence that object is a hand (double value between 0 and 1)
             */
            double classify(const std::vector<double> features);

            /**
            * Get the index of the SVM this classifier would use for a certain feature vector.
            * Useful for checking which SVM is working well and which is not
            * Note: 1 SVM is used for each number of visible fingers, i.e. 1 for hands with 1 visible finger,
            * 1 for hands with 2 visible fingers, etc.
            * @param features vector of features (index 0 should be number of fingers)
            * @return index of SVM used
            */
            static inline int getSVMIdx(const std::vector<double> features);

        private:
            /**
             * Maximum number of features. Any additonal features will be cut off.
             */
            static const int MAX_FEATURES = 54;

            // SVM storage
            libsvm::svm_model * model[NUM_SVMS];
            libsvm::svm_problem prob[NUM_SVMS];
            libsvm::svm_node * xstore[NUM_SVMS];
            libsvm::svm_parameter * param[NUM_SVMS];

            /**
            * Initialize classifiers
            */
            void initSVMs(const double hyperparams[5 * NUM_SVMS] = DEFAULT_HYPERPARAMS);
        };
    }
}
