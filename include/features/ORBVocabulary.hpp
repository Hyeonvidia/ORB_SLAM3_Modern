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

#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

// R3: the vocabulary moved to the recognition module — upstream DBoW2
// submodule + wrapper class (docs/REFACTOR_PLAN.md). This forwarder exists
// only for the byte-frozen backend_equiv reference snapshot, whose headers
// still include "features/ORBVocabulary.hpp"; live code includes
// "recognition/OrbVocabulary.hpp" directly.
#include "recognition/OrbVocabulary.hpp"

#endif // ORBVOCABULARY_H
