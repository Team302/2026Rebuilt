#pragma once
#include <string>

class IOrchestraPlayable
{
public:
    virtual void LoadMusic(const std::string &filePath) = 0;
    virtual void StartMusic() = 0;
    virtual void StopMusic() = 0;
    virtual ~IOrchestraPlayable() = default;
};
