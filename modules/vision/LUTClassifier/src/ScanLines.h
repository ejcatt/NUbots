/*
 * This file is part of ScanLines.
 *
 * ScanLines is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScanLines is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScanLines.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_SCANLINES_H
#define MODULES_VISION_SCANLINES_H

#include <nuclear> 
#include <string>
#include <armadillo>

#include "messages/input/Image.h"
#include "messages/support/Configuration.h"
#include "messages/vision/ClassifiedImage.h"

#include "LookUpTable.h"
#include "GreenHorizon.h"
#include "SegmentLogic.h"

namespace modules {
    namespace vision {

        /**
         * Generates horizontal and vertical scan lines.
         *
         * @author Alex Biddulph
         */
        class ScanLines {
        private:
            unsigned int HORIZONTAL_SCANLINE_SPACING;
            unsigned int VERTICAL_SCANLINE_SPACING;            

            /*! @brief Returns a std::vector of ColourSegments detailing the
            horizontal colour segments in the image.
            */
            std::vector<messages::vision::ColourSegment> classifyHorizontalScan(const messages::input::Image& image, unsigned int y, const LookUpTable& LUT);

            /*! @brief Returns a std::vector of ColourSegments detailing the
            vertical colour segments in the image.
            */
            std::vector<messages::vision::ColourSegment> classifyVerticalScan(const messages::input::Image& image, const arma::vec2& start, const LookUpTable& LUT);

        public:
            /*! @brief Loads configuration file.
            */ 
            ScanLines();
            
            void setParameters( unsigned int HORIZONTAL_SCANLINE_SPACING_,
                           unsigned int VERTICAL_SCANLINE_SPACING_) {
                HORIZONTAL_SCANLINE_SPACING = HORIZONTAL_SCANLINE_SPACING_;
                VERTICAL_SCANLINE_SPACING = VERTICAL_SCANLINE_SPACING_;
            }
            
            /*! @brief Generates the scan lines
            */ 
            std::vector<int> generateScanLines(const messages::input::Image& image, const GreenHorizon& greenHorizonPoints);

            /*! @brief Returns a std::vector of ColourSegments relating classified 
            horizontal scan lines.
            */
            messages::vision::SegmentedRegion classifyHorizontalScanLines(const messages::input::Image& originalImage, const std::vector<int>& horizontalScanLines, const LookUpTable& LUT);

            /*! @brief Returns a std::vector of ColourSegments relating to classified 
            vertical scan lines.
            */
            messages::vision::SegmentedRegion classifyVerticalScanLines(const messages::input::Image& originalImage, const GreenHorizon& greenHorizon, const LookUpTable& LUT);
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_SCANLINES_H
