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

#include "ScanLines.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::support::Configuration;
        using messages::vision::ClassifiedImage;
		using messages::vision::ColourSegment;
		using messages::vision::COLOUR_CLASS;
		using messages::vision::Colour;
		using messages::vision::SegmentedRegion;
		using messages::vision::ScanDirection;
        
 		ScanLines::ScanLines() {
 			// Empty Constructor.
 		}

		std::vector<int> ScanLines::generateScanLines(const Image& img, const GreenHorizon& greenHorizon) {
			std::vector<int> horizontalScanLines;
			int bottomHorizontalScan = img.height() - 1;														//we need h-scans under the GH for field lines
			const std::vector<arma::vec2>& horizonPoints = greenHorizon.getInterpolatedPoints();		// Need this to get the left and right
			std::cout << "Num Horizon points = " << horizonPoints.size()<< std::endl;
			arma::vec2 left = horizonPoints.front();
			arma::vec2 right = horizonPoints.back();

			if (left[1] >= img.height()) { // Element 1 is the y-component.
//				log<NUClear::WARN>("Left horizon limit exceeds image height: ", left[1]);
				std::cout << "Left horizon limit exceeds image height: " << left[1] << std::endl;
			}
			
			if (right[1] >= img.height()) { // Element 1 is the y-component.
//				log<NUClear::WARN>("Left horizon limit exceeds image height: ", right[1]);
				std::cout << "Left horizon limit exceeds image height: " << right[1] << std::endl;
			}
			
			const int SPACING = std::max(HORIZONTAL_SCANLINE_SPACING, 1U);

			for (int y = bottomHorizontalScan; y >= 0; y -= SPACING) {
				horizontalScanLines.push_back(y);
			}

			return horizontalScanLines;
		}

		SegmentedRegion ScanLines::classifyHorizontalScanLines(const Image& originalImage, const std::vector<int>& horizontalScanLines, const LookUpTable& LUT) {
			SegmentedRegion classifications;

		    for (const auto& scanLine : horizontalScanLines) {
				classifications.m_segmentedScans.push_back(classifyHorizontalScan(originalImage, scanLine, LUT));
			}

			classifications.m_direction = ScanDirection::HORIZONTAL;

			return classifications;
		}

		SegmentedRegion ScanLines::classifyVerticalScanLines(const Image& originalImage, const GreenHorizon& greenHorizon, const LookUpTable& LUT) {
			
			const std::vector<arma::vec2>& verticalStartPoints = greenHorizon.getInterpolatedSubset(VERTICAL_SCANLINE_SPACING);
			SegmentedRegion classifications;

		    for (const auto& scanLine : verticalStartPoints) {
				classifications.m_segmentedScans.push_back(classifyVerticalScan(originalImage, scanLine, LUT));
			}
    		classifications.m_direction = ScanDirection::VERTICAL;
			return classifications;
		}

        std::vector<ColourSegment> ScanLines::classifyHorizontalScan(const Image& image, unsigned int y, const LookUpTable& LUT) {
			std::vector<ColourSegment> result;
			arma::vec2 startPoint, endPoint;

			if (y >= image.height()) {
//				log<NUClear::ERROR>("ScanLines::classifyHorizontalScan invalid y: ", y);
				std::cout << "ScanLines::classifyHorizontalScan invalid y: " <<  y << std::endl;
				return result;
			}

			//simple and nasty first
			//Colour previous, current, next
			unsigned int startPosition = 0, x;
			Colour startColour = LUT.classifyPixel(image(0, y));
			Colour currentColour;
			ColourSegment segment;

			for (x = 0; x < image.width(); x++) {
				currentColour = LUT.classifyPixel(image(x, y));

				if (currentColour != startColour) {
					//start of new segment
					//make new segment and push onto std::vector
					startPoint[0] = startPosition;
					startPoint[1] = y;
					endPoint[0] = x;
					endPoint[1] = y;
					SegmentLogic::setColourSegment(segment,startPoint, endPoint, startColour);
					result.push_back(segment);

					//start new segment
					startColour = currentColour;
					startPosition = x;
				}
			}

			startPoint[0] = startPosition;
			startPoint[1] = y;
			endPoint[0] = x - 1;
			endPoint[1] = y;
			SegmentLogic::setColourSegment(segment,startPoint, endPoint, startColour);
			result.push_back(segment);

			return result;
		}

		std::vector<ColourSegment> ScanLines::classifyVerticalScan(const Image& image, const arma::vec2& start, const LookUpTable& LUT) {
			std::vector<ColourSegment> result;
			arma::vec2 startPoint, endPoint;

			if ((start[1] >= image.height()) || (start[1] < 0) || (start[0] >= image.width()) || (start[0] < 0)) {
//				log<NUClear::ERROR>("ScanLines::classifyVerticalScan invalid start position: ", start);
				std::cout << "ScanLines::classifyVerticalScan invalid start position: " << start << std::endl;
				return result;
			}

			//simple and nasty first
			//Colour previous, current, next
			Colour startColour = LUT.classifyPixel(image(start[0], start[1])), currentColour;
			ColourSegment segment;
			unsigned int startPosition = start[1], x = start[0], y;

			for (y = start[1]; y < image.height(); y++) {
				currentColour = LUT.classifyPixel(image(x, y));

				if (currentColour != startColour) {
					//start of new segment
					//make new segment and push onto std::vector
					startPoint[0] = x;
					startPoint[1] = startPosition;
					endPoint[0] = x;
					endPoint[1] = y;
					SegmentLogic::setColourSegment(segment,startPoint, endPoint, startColour);
					result.push_back(segment);

					//start new segment
					startColour = currentColour;
					startPosition = y;
				}
			}

			startPoint[0] = x;
			startPoint[1] = startPosition;
			endPoint[0] = x;
			endPoint[1] = y;
			SegmentLogic::setColourSegment(segment,startPoint, endPoint, startColour);
			result.push_back(segment);

			return result;
		}
	}
}