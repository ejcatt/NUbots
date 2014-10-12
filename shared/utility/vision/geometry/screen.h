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
#ifndef UTILITY_MATH_VISION_H
#define UTILITY_MATH_VISION_H

#include <cmath>
#include <armadillo>
#include <nuclear>
#include "messages/input/Image.h"

namespace utility {
namespace vision {
namespace geometry {
    
    using messages::input::Image;
    
    inline arma::mat snapToScreen(const arma::mat& rayPositions, const arma::vec& rayLength, const Image& image) {
        //returns a mat of vectors of resized rayLength so that rays end at the edge of the image space
        arma::vec scales(rayPositions.n_rows,1);
        if (image.lens.type == Image::Lens::Type::RADIAL) {
            const double pixelFOV = image.lens.parameters.radialFOV/image.lens.parameters.pixelPitch;
            
            if (rayLength[0] != 0 and rayLength[1] != 0) {
                const arma::mat k = ( rayPositions )/arma::repmat(rayLength.t(),1,rayPositions.n_rows);
                
                const arma::vec b = 2.0*(k.col(0) + k.col(1));
                
                const arma::vec sqrtfactor =    arma::sqrt(arma::square(b)-4.0*2.0*arma::sum(arma::square(k),1));
                
                scales = (-b+sqrtfactor)/4.0;
            } else if (rayLength[1] != 0) {
                const arma::mat k = ( rayPositions.col(1) )/arma::repmat(rayLength.row(1),1,rayPositions.n_rows);
                
                const arma::vec b = 2.0*k;
                
                const arma::vec sqrtfactor =    arma::sqrt(arma::square(b)-4.0*arma::sum(arma::square(k),1));
                
                scales = (-b+sqrtfactor)/2.0;
            } else if (rayLength[0] != 0) {
                const arma::mat k = ( rayPositions.col(0) )/arma::repmat(rayLength.row(0),1,rayPositions.n_rows);
                
                const arma::vec b = 2.0*k;
                
                const arma::vec sqrtfactor =    arma::sqrt(arma::square(b)-4.0*arma::sum(arma::square(k),1));
                
                scales = (-b+sqrtfactor)/2.0;
            }
            
            
        } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
            
            //check which direction to calculate the x edge of image intercept from
            if (rayLength[0] > 0) { 
                //calculate the positive x intercept
                scales = (image.dimensions[0]/2-rayPositions.col(0))/rayLength[0];
                
            } else if (rayLength[0] < 0) {
                //calculate the positive x intercept
                scales = (rayPositions.col(0)-image.dimensions[0]/2)/rayLength[0];
                
            }
            
            //now do y edge of image intercepts
            if (rayLength[0] == 0) {
                if (rayLength[1] > 0) { 
                    //calculate the positive x intercept
                    scales = (image.dimensions[1]/2-rayPositions.col(1))/rayLength[1];
                    
                } else if (rayLength[1] < 0) {
                    //calculate the positive x intercept
                    scales = (rayPositions.col(1)-image.dimensions[1]/2)/rayLength[1];
                    
                }
            } else {
                if (rayLength[1] > 0) { 
                    //calculate the positive x intercept
                    scales = arma::min( (image.dimensions[1]/2-rayPositions.col(1))/rayLength[1], scales);
                    
                } else if (rayLength[1] < 0) {
                    //calculate the positive x intercept
                    scales = arma::min( (rayPositions.col(1)-image.dimensions[1]/2)/rayLength[1], scales);
                    
                }
            }
        }
        
        return arma::round( arma::repmat(rayLength,scales.n_rows,1) % arma::repmat(scales,1,2) );
    }
    
    inline arma::imat trimToImage(const arma::imat& pixels, const Image& image) {
        //remove any pixel references not within the image area
        arma::imat result = pixels(arma::find( (pixels.col(1) >= 0) % (pixels.col(1) < image.dimensions[0]) ));
        result = result.rows(arma::find( (result.col(0) >= 0) % (result.col(0) < image.dimensions[1]) ));
        return std::move(result);
    }
    
    inline arma::mat trimToFOV(const arma::mat& rays, const Image& image) {
        //remove any pixel references not within the screen FOV
        arma::ivec selected;
        if (image.lens.type == Image::Lens::Type::RADIAL) {
            //assuming Z is the forward vector
            const double fsize = arma::cos(image.lens.parameters.radialFOV/2);
            selected = arma::find(rays.col(2) < fsize);
            
        } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
            const double fsizex = arma::cos(image.lens.parameters.FOV[0]/2);
            const double fsizey = arma::cos(image.lens.parameters.FOV[1]/2);
            selected = arma::find( (rays.col(0) < fsizex) % (rays.col(1) < fsizey) );
        }
        
        return std::move(rays.rows(selected));
    }
    
    inline arma::imat bulkRay2Pixel(const arma::mat& rays, const Image& image) {
        //convert camera rays to pixels
        arma::mat result;
        
        if (image.lens.type == Image::Lens::Type::RADIAL) {
            
            result = rays.cols(0,1);
            
            arma::vec rads = arma::acos(rays.col(2));
            
            result /= arma::repmat(arma::sqrt(arma::sum(arma::square(result),1))*image.lens.parameters.pixelPitch,2,1);
            result *= arma::repmat(rads,2,1);
            result += arma::repmat(image.lens.parameters.imageCenter,1,rays.n_rows);
        
        } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
            
            arma::vec2 imageCenter = arma::vec2({image.dimensions[1]/2,
                                                 image.dimensions[0]/2});
            
            result = rays.cols(0,1);
            
            result *= arma::repmat(image.lens.parameters.focalLengthPixels/rays.col(2),2,1);
            
            result += arma::repmat(imageCenter,1,rays.n_rows);
        }
        
        
        return arma::conv_to<arma::imat>::from(arma::round(result));
    }
    
    inline arma::mat bulkPixel2Ray(const arma::imat& pixels, const Image& image) {
        //convert a matrix of rows of 2d pixels into spherical camera rays
        
        arma::mat result(pixels.n_rows,3);
        if (image.lens.type == Image::Lens::Type::RADIAL) {
            
            //center the pixels
            const arma::mat px = arma::conv_to<arma::mat>::from(pixels) - 
                                 arma::repmat(image.lens.parameters.imageCenter.t(),1,pixels.n_rows) *
                                 image.lens.parameters.pixelPitch;
            
            //get all the radian values
            arma::vec rads = arma::sqrt(
                                arma::sum(
                                   arma::square(
                                      px),1));
            
            arma::vec sinRadsOnRads = arma::sin(rad)/rads;
            
            //project to the unit sphere
            result.col(0) = px.col(0) % sinRadsOnRads;
            result.col(1) = px.col(1) % sinRadsOnRads;
            result.col(2) = arma::cos(rads);
            
        } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
            
            arma::vec2 imageCenter = arma::vec2({image.dimensions[1]/2,
                                                 image.dimensions[0]/2});
            
            result.cols(0,1) = arma::conv_to<arma::mat>::from(pixels) - 
                                 arma::repmat(imageCenter.t(),1,pixels.n_rows);
            result.cols(2) = image.lens.parameters.focalLength;
            
            result /= arma::repmat(arma::sqrt(arma::sum(arma::square(result),1)),3,1);
        }
    
        return std::move(result);
    }
    
    
}
}
}

#endif