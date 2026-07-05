#ifndef BEHAVIORTREELOADER_HPP
#define BEHAVIORTREELOADER_HPP

#include <filesystem>
#include <string>

#include <plog/Log.h>
#include "behaviortree_cpp/bt_factory.h"

namespace BehaviorTreeLoader
{
    // registers every *.xml tree definition found in aDirectory with the factory.
    // trees can reference each other via <SubTree ID="..."/> regardless of which
    // file they were defined in, as long as all files are registered before a
    // tree is instantiated with factory.createTree(...).
    inline void registerTreesFromDirectory(BT::BehaviorTreeFactory& aFactory, const std::string& aDirectory)
    {
        if(!std::filesystem::exists(aDirectory) || !std::filesystem::is_directory(aDirectory))
        {
            LOGE << "Behavior tree directory does not exist: " << aDirectory;
            return;
        }

        for(const auto& entry : std::filesystem::directory_iterator(aDirectory))
        {
            if(entry.is_regular_file() && entry.path().extension() == ".xml")
            {
                LOGD << "Registering behavior tree file: " << entry.path().string();
                aFactory.registerBehaviorTreeFromFile(entry.path().string());
            }
        }
    }
}

#endif // BEHAVIORTREELOADER_HPP
