/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#ifndef UTILITY_VISION_ORB_FEATURE_EXTRACTOR_H
#define UTILITY_VISION_ORB_FEATURE_EXTRACTOR_H

#include "messages/input/Image.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/support/Configuration.h"

#include <armadillo>

namespace utility {
	namespace vision {
		class ORBFeatureExtractor{
		public:

			class ExtractedFeature {
			public:
				arma::vec screenAngular;	//Compulsory
			};

			ORBFeatureExtractor();
			std::vector<ExtractedFeature> extractFeatures(const messages::input::Image& image, const messages::localisation::Self& self, const messages::input::Sensors& sensors);

			void setParameters(const messages::support::Configuration<ORBFeatureExtractor>& config);
			//Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
            //Order of vector is strongest to weakest
            //Add new features here to the feature list and pick up missing filters and strengths below
			std::vector<std::tuple<int, int, float>> matchFeatures(std::vector<ExtractedFeature>& features, 
																   const std::vector<ExtractedFeature>& newFeatures,
																   size_t MAX_MATCHES);
            static constexpr const char* CONFIGURATION_PATH = "ORBFeatureExtractor.json";

		};
	}
}

#endif