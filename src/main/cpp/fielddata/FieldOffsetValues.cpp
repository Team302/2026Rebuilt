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
#include "fielddata/FieldOffsetValues.h"
#include "fielddata/FieldConstants.h"

//------------------------------------------------------------------
/// @brief Singleton instance pointer - initialized to nullptr
//------------------------------------------------------------------
FieldOffsetValues *FieldOffsetValues::m_instance = nullptr;

//------------------------------------------------------------------
/// @brief      Get the singleton instance of FieldOffsetValues
/// @return     FieldOffsetValues* - Pointer to the singleton instance
/// @details    Creates the instance on first call. Subsequent calls return
///             the same instance, ensuring only one FieldOffsetValues object
///             exists throughout the program lifetime
//------------------------------------------------------------------
FieldOffsetValues *FieldOffsetValues::GetInstance()
{
    if (FieldOffsetValues::m_instance == nullptr)
    {
        FieldOffsetValues::m_instance = new FieldOffsetValues();
    }
    return FieldOffsetValues::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for FieldOffsetValues
/// @details    Initializes the X-coordinate offsets for both alliance depots
///             and outposts by retrieving their positions from field constants.
///             - Retrieves blue and red depot neutral side X positions
///             - Sets outpost X positions equal to depot X positions
///             (indicating outposts are aligned with depots along the X axis)
//------------------------------------------------------------------
FieldOffsetValues::FieldOffsetValues()
{
    auto fieldConstants = FieldConstants::GetInstance();

    if (fieldConstants != nullptr)
    {
        m_blueDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE).X() + units::length::meter_t{DEPOT_OFFSET};
        m_redDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE).X() - units::length::meter_t{DEPOT_OFFSET};

        m_blueHubX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER).X() - HUB_OFFSET;
        m_redHubX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER).X() + HUB_OFFSET;
    }
    else
    {
        // Fallback to safe default values if FieldConstants is unavailable
        m_blueDepotX = units::length::meter_t{0.0};
        m_redDepotX = units::length::meter_t{0.0};
        m_blueHubX = units::length::meter_t{0.0};
        m_redHubX = units::length::meter_t{0.0};
    }

    m_blueOutpostX = m_blueDepotX;
    m_redOutpostX = m_redDepotX;
}

//------------------------------------------------------------------
/// @brief      Get the X offset value for a specific field item
/// @param      isRedSide - true for red alliance, false for blue alliance
/// @param      item - The field offset item to retrieve (OUTPOST_X or DEPOT_X)
/// @return     units::length::meter_t - The X-coordinate offset in meters
/// @details    Returns alliance-specific X-coordinate offsets for either
///             the outpost or depot. The function uses ternary operators to
///             select the correct value based on the alliance side:
///             - For OUTPOST_X: returns red outpost X if red side, else blue outpost X
///             - For DEPOT_X: returns red depot X if red side, else blue depot X
//------------------------------------------------------------------
units::length::meter_t FieldOffsetValues::GetXValue(bool isRedSide, FIELD_OFFSET_ITEMS item) const
{
    if (item == FIELD_OFFSET_ITEMS::OUTPOST_X)
    {
        return isRedSide ? m_redOutpostX : m_blueOutpostX;
    }
    else if (item == FIELD_OFFSET_ITEMS::DEPOT_X)
    {
        return isRedSide ? m_redDepotX : m_blueDepotX;
    }
    else if (item == FIELD_OFFSET_ITEMS::HUB_X)
    {
        return isRedSide ? m_redHubX : m_blueHubX;
    }
    else
    {
        // Handle invalid item case (could throw an exception or return a default value)
        return units::length::meter_t{0.0}; // Default fallback value
    }
}