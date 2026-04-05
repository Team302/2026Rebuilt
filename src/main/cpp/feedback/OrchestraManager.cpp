//====================================================================================================================================================
// Copyright 2026 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include "feedback/OrchestraManager.h"
#include <filesystem>
#include <algorithm>

static bool g_orchestraManagerDestroyed = false;

IOrchestraPlayable::~IOrchestraPlayable()
{
    // Auto-unregister upon destruction to prevent dangling pointers
    if (!g_orchestraManagerDestroyed && OrchestraManager::GetInstance() != nullptr)
    {
        OrchestraManager::GetInstance()->Unregister(this);
    }
}

OrchestraManager *OrchestraManager::GetInstance()
{
    static OrchestraManager instance;
    return &instance;
}

OrchestraManager::~OrchestraManager()
{
    g_orchestraManagerDestroyed = true;
}

void OrchestraManager::Register(IOrchestraPlayable *playable)
{
    if (playable != nullptr)
    {
        if (std::find(m_playables.begin(), m_playables.end(), playable) == m_playables.end())
        {
            m_playables.push_back(playable);
        }
    }
}

void OrchestraManager::Unregister(IOrchestraPlayable *playable)
{
    if (playable != nullptr)
    {
        m_playables.erase(std::remove(m_playables.begin(), m_playables.end(), playable), m_playables.end());
    }
}

void OrchestraManager::LoadMusic(const std::string &filePath)
{
    if (!std::filesystem::exists(filePath))
    {
        return;
    }

    // Create a copy of the vector to safely iterate, avoiding dangling pointers
    auto playables = m_playables;
    for (auto playable : playables)
    {
        if (playable != nullptr)
        {
            playable->LoadMusic(filePath);
            m_musicLoaded = true;
        }
    }
}

void OrchestraManager::StartMusic()
{
    if (m_musicLoaded)
    {
        // Create a copy of the vector to safely iterate, avoiding dangling pointers
        auto playables = m_playables;
        for (auto playable : playables)
        {
            if (playable != nullptr)
            {
                playable->StartMusic();
            }
        }
    }
}

void OrchestraManager::StopMusic()
{
    // Create a copy of the vector to safely iterate, avoiding dangling pointers
    auto playables = m_playables;
    for (auto playable : playables)
    {
        if (playable != nullptr)
        {
            playable->StopMusic();
        }
    }
}
