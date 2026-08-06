/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORB_SLAM3_CAMERA_FACTORY_HPP
#define ORB_SLAM3_CAMERA_FACTORY_HPP

#include "camera/GeometricCamera.hpp"

#include <string>
#include <vector>

namespace ORB_SLAM3
{

// Single creation seam for camera models (P3-2). Ownership note: the caller
// owns the raw pointer today, matching the surrounding code; instances handed
// to Atlas::AddCamera may be deduplicated there (an equal camera already in
// the atlas is returned instead). Boost-serialization loading still constructs
// cameras directly via register_type — that path bypasses any factory by
// design and is unaffected.
class CameraFactory
{
public:
    // "PinHole"/"Rectified" -> Pinhole (4 params: fx fy cx cy)
    // "KannalaBrandt8"      -> KannalaBrandt8 (8 params: fx fy cx cy k0..k3)
    // Unknown model name returns nullptr.
    static GeometricCamera* create(const std::string& model,
                                   const std::vector<float>& params);
};

}

#endif // ORB_SLAM3_CAMERA_FACTORY_HPP
