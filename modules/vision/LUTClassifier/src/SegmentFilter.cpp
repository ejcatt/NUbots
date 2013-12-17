/*
 * This file is part of SegmentFilter.
 *
 * SegmentFilter is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SegmentFilter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SegmentFilter.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "SegmentFilter.h"

namespace modules {
    namespace vision {
    	using messages::vision::ClassifiedImage;
    	using messages::vision::ColourSegment;
    	using messages::vision::SegmentedRegion;
    	using messages::vision::COLOUR_CLASS;
    	using messages::vision::ScanDirection;

		SegmentFilter::SegmentFilter() {			
		}

		std::unique_ptr<messages::vision::ClassifiedImage> SegmentFilter::classifyImage(const SegmentedRegion& horizontalSegments, const SegmentedRegion& verticalSegments) const {
			
			SegmentedRegion horizontalFiltered, verticalFiltered;
			std::map<COLOUR_CLASS, std::vector<ColourSegment>> horizontalResult, verticalResult;
			
			//Construct image for emission
			std::unique_ptr<ClassifiedImage> image = std::unique_ptr<ClassifiedImage>(new ClassifiedImage());

			if (PREFILTER_ON) {
				preFilter(horizontalSegments, horizontalFiltered);
				preFilter(verticalSegments, verticalFiltered);

		        image->horizontal_filtered_segments = horizontalFiltered;
		        image->vertical_filtered_segments = verticalFiltered;
			} else {
				image->horizontal_filtered_segments = horizontalSegments;
		        image->vertical_filtered_segments = verticalSegments;
			}
			filter(horizontalFiltered, horizontalResult);
			filter(verticalFiltered, verticalResult);

			
			image->matched_horizontal_segments = horizontalResult;
			image->matched_vertical_segments = verticalResult;
			return std::move(image);
		}

		void SegmentFilter::preFilter(const SegmentedRegion& scans, SegmentedRegion &result) const {
			const std::vector<std::vector<ColourSegment>>& segments = scans.m_segmentedScans;
			std::vector<std::vector<ColourSegment>> finalSegments;
			std::vector<ColourSegment> line;
		
			std::vector<std::vector<ColourSegment>>::const_iterator line_it;
			std::vector<ColourSegment>::const_iterator before_it, middle_it, after_it;
			ScanDirection dir = scans.m_direction;
		
			//loop through each scan
			for(line_it = segments.begin(); line_it < segments.end(); line_it++) {
				if(line_it->size() >= 3) {
				    //move down segments in triplets replacing the middle if necessary
				    before_it = line_it->begin();
				    middle_it = before_it + 1;
				    line.clear();
				    line.push_back(*before_it);         //add the first segment
				    
				    for(after_it = before_it + 2; after_it < line_it->end(); after_it++) {
				        applyReplacements(*before_it, *middle_it, *after_it, line, dir);
				        before_it = middle_it;
				        middle_it = after_it;
				    }
				    
				    line.push_back(line_it->back());    //add the last segment
				    joinMatchingSegments(line);         //merge any now matching segments
				    finalSegments.push_back(line);
				}
				
				else {
				    //push the unfiltered line into the result as it is too small to filter
				    line.assign(line_it->begin(), line_it->end());
				    finalSegments.push_back(line);
				}
			}
	
			// Store the result of the pre-filtering.
			result.m_segmentedScans = finalSegments;								//vector assignment operator copies elements
			result.m_direction = dir;
		}

		void SegmentFilter::filter(const SegmentedRegion &scans, std::map<COLOUR_CLASS, std::vector<ColourSegment>>& result) const {
			switch (scans.m_direction) {
				case ScanDirection::VERTICAL: {
					for (auto rule : m_verticalRules) {
						std::vector<ColourSegment>& segments = result[rule.getColourClass()];
						checkRuleAgainstRegion(scans, rule, segments);
					}
					
					break;
				}
			
				case ScanDirection::HORIZONTAL: {
					for (auto rule : m_horizontalRules) {
						std::vector<ColourSegment>& segments = result[rule.getColourClass()];
						checkRuleAgainstRegion(scans, rule, segments);
					}
				
					break;
				}
			
				default: {
					std::cout << "SegmentFilter::filter - invalid direction" << std::endl;
					return;
				}
			}   
		}

		void SegmentFilter::checkRuleAgainstRegion(const SegmentedRegion& scans, const ColourTransitionRule& rule, std::vector<ColourSegment>& matches) const {
			const std::vector<std::vector<ColourSegment>>& segments = scans.m_segmentedScans;
			std::vector<ColourSegment>::const_iterator it;

			//loop through each scan
			for (auto vs : segments) {
				// Only check for multiple segments
				if (vs.size() > 1) {
				    // Move down segments in scan pairwise
				    it = vs.begin();
				    
				    // First check start pair alone
				    if (rule.match(ColourTransitionRule::nomatch, *it, *(it + 1))) {
				        matches.push_back(*it);
				    }
				    
				    it++;
				    
				    // Then check the rest in triplets
				    while (it < vs.end() - 1) {
				        if (rule.match(*(it - 1), *it, *(it + 1))) {
				            matches.push_back(*it);
				        }
				        
				        it++;
				    }
				    
				    // Lastly check final pair alone
				    if (rule.match(*(it - 1), *it, ColourTransitionRule::nomatch)) {
				        matches.push_back(*it);
				    }
				}
			}
		}

		void SegmentFilter::applyReplacements(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after, std::vector<ColourSegment>& replacements, ScanDirection dir) const {
			std::vector<ColourReplacementRule>::const_iterator rules_it, begin, end;
			ColourSegment tempSegment;
		
			switch(dir) {
				case ScanDirection::VERTICAL: {
					begin = m_verticalReplacementRules.end();
					end = m_verticalReplacementRules.end();
					
					break;
				}
				
				case ScanDirection::HORIZONTAL: {
					begin = m_horizontalReplacementRules.begin();
					end = m_horizontalReplacementRules.end();
					
					break;
				}
				
				default: {
					std::cout << "SegmentFilter::applyReplacements - invalid direction" << std::endl;
					return;
				}
			}    
		
			tempSegment = middle;
		
			for(rules_it = begin; rules_it < end; rules_it++) {
				if (rules_it->match(before, middle, after)) {
				    // Replace middle using replacement method.
				    switch (rules_it->getMethod()) {
						case ColourReplacementRule::BEFORE: {
							tempSegment.m_colour = before.m_colour;
							replacements.push_back(tempSegment);

							break;
						}
						
						case ColourReplacementRule::AFTER: {
							tempSegment.m_colour = after.m_colour;
							replacements.push_back(tempSegment);
							
							break;
						}
						
						case ColourReplacementRule::SPLIT: {
							// Generate two new segments matching each end and push them both back.
							arma::vec2 start_pt	= tempSegment.m_start;
							arma::vec2 end_pt	= tempSegment.m_end;
							arma::vec2 mid_pt	= arma::vec2((start_pt + end_pt) * 0.5);
							
							SegmentLogic::setColourSegment(tempSegment,start_pt, mid_pt, before.m_colour);
							replacements.push_back(tempSegment);
							
							SegmentLogic::setColourSegment(tempSegment,mid_pt, end_pt, after.m_colour);
							replacements.push_back(tempSegment);
							
							break;
						}
						
						case ColourReplacementRule::INVALID: {
							std::cout << "SegmentFilter::applyReplacements - invalid replacement rule" << std::endl;
							replacements.push_back(middle);							
							break;
						}
					}
					
					// Replacements found so exit.
					return;
				}
			}
		
			replacements.push_back(middle); //no replacement so keep middle
		}
		
		void SegmentFilter::joinMatchingSegments(std::vector<ColourSegment>& line) const {
			std::vector<ColourSegment>::iterator before_it, after_it;
			before_it = line.begin();
			after_it = before_it + 1;
			
			while(after_it<line.end()) {
				if(before_it->m_colour == after_it->m_colour) {
				    SegmentLogic::joinColourSegment(*before_it,*after_it);
				    after_it = line.erase(after_it);
				    before_it = after_it - 1;
				}
				
				else {
				    after_it++;
				    before_it++;
				}
			}
		}
		
		void SegmentFilter::clearRules(){
			m_horizontalReplacementRules.clear();
			m_verticalReplacementRules.clear();
			m_horizontalRules.clear();
			m_verticalRules.clear();
		}
		void SegmentFilter::addTransitionRule(const ColourTransitionRule& rule){
			m_horizontalRules.push_back(rule);
			m_verticalRules.push_back(rule);
		}

		void SegmentFilter::addReplacementRule(const ColourReplacementRule& rule){
			m_horizontalReplacementRules.push_back(rule);
			m_verticalReplacementRules.push_back(rule);
		}
	}
}
