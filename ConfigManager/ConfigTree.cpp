/*! 
	@file 	ConfigTree.cpp
    @brief 	Implementation file for the ConfigTree class. 
    
    @author Sophie Calland, Mitchell Metcalfe
    
  Copyright (c) 2012 Sophie Calland
    
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <string>
#include <exception>

#include "ConfigStorageManager.h"
#include "ConfigTree.h"

using std::string;
// using ConfigSystem::ConfigStorageManager;

using boost::property_tree::ptree;

namespace ConfigSystem
{
	ConfigTree::ConfigTree(ptree root)
    {
        CONFIGSYS_DEBUG_CALLS;

        _treeRoot = root;
    }

    ConfigTree::~ConfigTree()
    {
        CONFIGSYS_DEBUG_CALLS;
    }


    std::string ConfigTree::makeFullPath(const std::string paramPath, const std::string paramName)
    {
        return paramPath + "." + paramName;
    }

	bool ConfigTree::getParam (
            const std::string paramPath, 
            const std::string paramName, 
            ConfigParameter &data
            )
	{
		CONFIGSYS_DEBUG_CALLS;

        // Indicates successful retrieval
        bool success = false;

        // Create the full path to the desired parameter
        std::string fullPath = makeFullPath(paramPath, paramName);

		try
		{
            // Get the subtree representing the desired parameter.
            ptree paramSubtree = _treeRoot.get_child(fullPath);

            // Convert the parameter subtree into a parameter object.
            success = paramFromPtree(paramSubtree, data);
		}
		catch (boost::property_tree::ptree_error e)
		{
			std::cout   << "ConfigTree::getParam(...): ACCESS ERROR:" 
                        << e.what() 
                        << std::endl;
            return false;
		}

		return success;
	}

    bool ConfigTree::storeParam (
            const std::string paramPath, 
            const std::string paramName, 
            ConfigParameter data
            )
    {
        CONFIGSYS_DEBUG_CALLS;
        
        // Indicates successful storage
        bool success = false;

        // Create the full path to the desired parameter
        std::string fullPath = makeFullPath(paramPath, paramName);

        try
        {
            // Convert the parameter object into a parameter subtree.
            ptree paramSubtree;
            success = ptreeFromParam(data, paramSubtree);
            if(!success) return false;
            
            // Put the subtree representing the converted parameter into the
            // config tree.
            _treeRoot.put_child(fullPath, paramSubtree);
        }
        catch (boost::property_tree::ptree_error e)
        {
            std::cout   << "ConfigTree::getParam(...): ACCESS ERROR:" 
                        << e.what() 
                        << std::endl;
            return false;
        }

        return success;
    }

    bool ConfigTree::paramFromPtree(ptree fromPtree, ConfigParameter &toParam)
    {
        CONFIGSYS_DEBUG_CALLS;

        std::string typStr = fromPtree.get<std::string>("type");
        value_type vt = stringToValueType(typStr);
        
        toParam = ConfigParameter(vt);

        toParam.setDescription(fromPtree.get("desc", ""));

        std::string modStr = fromPtree.get("modified", "false");
        toParam.setModified(modStr.compare("true") == 0);

        std::string lockStr = fromPtree.get("locked", "false");
        toParam.setLocked(lockStr.compare("true") == 0);

        addPtreeValueandRangeToParam(fromPtree, toParam);

        return true;
    }

    bool ConfigTree::ptreeFromParam(ConfigParameter fromParam, ptree &toPtree)
    {
        CONFIGSYS_DEBUG_CALLS;

        toPtree = ptree();

        toPtree.put("desc", fromParam.getDescription());
        
        std::string modStr = (fromParam.isModified())? "true" : "false";
        toPtree.put("modified", modStr);

        std::string lockStr = (fromParam.isLocked())? "true" : "false";
        toPtree.put("locked", lockStr);


        value_type vt = fromParam.getType();
        std::string typStr = makeValueTypeString(vt);
        toPtree.put("type", typStr);
        
        addParamValueandRangeToPtree(fromParam, toPtree);

        return true;
    }

    bool ConfigTree::addPtreeValueandRangeToParam(ptree fromPtree, ConfigParameter &toParam)
    {
        std::string typStr = fromPtree.get<std::string>("type");
        value_type vt = stringToValueType(typStr);
        std::cout << vt << std::endl;

        switch(vt)
        {
		    case vt_string: 
		    	addValueToParam<std::string>(fromPtree, toParam); 
		    	break;
		    	
		    case vt_double: 
		    	addValueToParam<double>(fromPtree, toParam); 
		    	break;
		    
		    case vt_1dvector_double:
		    	addVectorValueToParam1D<double>(fromPtree, toParam);
		    	break;
		    
		    case vt_bool: 
		    	addValueToParam<bool>(fromPtree, toParam); 
		    	break;
		    	
		    case vt_long: 
		    	addValueToParam<long>(fromPtree, toParam); 
		    	break;
		    
		    case vt_1dvector_long:
		    	addVectorValueToParam1D<long>(fromPtree, toParam);
		    	break;
		    
		    break;
        }
 
        switch(vt)
        {
        	//cases fall through as they use the same - change later if necessary.
		    case vt_double         :
		    case vt_1dvector_double: 
		        addRangeToParam<double>(fromPtree, toParam);
		        break;
		        
		    case vt_bool         :
		    case vt_long         :		    
		    case vt_1dvector_long: 
		        addRangeToParam<long>(fromPtree, toParam);
		        break;
        }
    }

    bool ConfigTree::addParamValueandRangeToPtree(ConfigParameter fromParam, ptree &toPtree)
    {
        value_type vt = fromParam.getType();
        
        // Add value
        switch(vt)
        {
		    case vt_string: 
		        addValueToPtree<std::string> (fromParam, toPtree); 
		        break;
		        
		    case vt_double: 
		        addValueToPtree<double> (fromParam, toPtree); 
		        break;
		        
		    case vt_1dvector_double:
		        addVectorValueToPtree1D<double>(fromParam, toPtree); 
		        break;
		        
		    case vt_bool           : 
		        addValueToPtree<bool> (fromParam, toPtree); 
		        break;
		        
		    case vt_long           : 
		        addValueToPtree<long> (fromParam, toPtree); 
		        break;
		        
		    case vt_1dvector_long  : 
		        addVectorValueToPtree1D<long>(fromParam, toPtree); 
		        break;
        }

        // Add range
        switch(vt)
        {
		    case vt_double         :
		    case vt_1dvector_double:
		        addRangeToPtree<double>(fromParam, toPtree);
		        break;
		    case vt_bool         :
		    case vt_long         :
		    case vt_1dvector_long:
		    	std::cout << "IN HERE" << std::endl;
		        addRangeToPtree<long>(fromParam, toPtree);
		        break;
        }
    }

// "Camera": {
//     "Sharpness": {
//         "desc": "sharpness",
//         "range": {
//             "min": "0",
//             "max": "255",
//             "lBound": "CLOSED",
//             "outside": "false",
//             "uBound": "CLOSED"
//         },
//         "value": "150",
//         "type": "float"
//     },

    
    ptree ConfigTree::getRoot()
    {
        return _treeRoot;
    }
}
