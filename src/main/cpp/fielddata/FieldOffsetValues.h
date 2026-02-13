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
#pragma once

// C++ includes

#include "units/length.h"

//====================================================================================================================================================
/// @enum FIELD_OFFSET_ITEMS
/// @brief Enumeration of field element offset items available for retrieval
///
/// Specifies which type of field element offset to retrieve using GetXValue()
//====================================================================================================================================================
enum class FIELD_OFFSET_ITEMS
{
    OUTPOST_X, ///< X-coordinate offset from the outpost
    DEPOT_X,   ///< X-coordinate offset from the depot neutral side
    HUB_X,     ///< X-coordinate offset from the hub center
    BUMP_X     ///< X-coordinate offset from the bump center
};

//====================================================================================================================================================
/// @class FieldOffsetValues
/// @brief Singleton class for managing field element offset values (depots and outposts)
///
/// This singleton class provides alliance-aware access to X-coordinate offsets for field elements
/// on the 2026 game field, specifically depots and outposts. It stores and retrieves position
/// data for both the red and blue alliance variants of these field elements.
///
/// Key responsibilities:
/// - Store X-coordinate offsets for blue and red depot neutral sides
/// - Store X-coordinate offsets for blue and red outposts
/// - Provide alliance-aware getter methods to retrieve offsets for navigation and positioning
///
/// The offsets are initialized from FieldConstants during object construction and used throughout
/// the robot code for autonomous and teleop navigation to field elements.
//====================================================================================================================================================
class FieldOffsetValues
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of FieldOffsetValues
    /// @return     FieldOffsetValues* - Pointer to the singleton instance
    //------------------------------------------------------------------
    static FieldOffsetValues *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Get the X offset value for a specific field element
    /// @param      isRedSide - true to get red alliance value, false for blue alliance
    /// @param      item - The field offset item to retrieve (OUTPOST_X or DEPOT_X)
    /// @return     units::length::meter_t - The X-coordinate offset in meters
    /// @details    Returns alliance-specific X-coordinate offsets for either the
    ///             outpost or depot based on the isRedSide parameter and item type.
    ///             - When item is OUTPOST_X: returns red or blue outpost X position
    ///             - When item is DEPOT_X: returns red or blue depot neutral side X position
    //------------------------------------------------------------------
    units::length::meter_t GetXValue(bool isRedSide, FIELD_OFFSET_ITEMS item) const;

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes the offset values by querying FieldConstants
    ///             for the positions of both blue and red depot neutral sides.
    ///             Sets outpost offsets equal to depot offsets since they are
    ///             aligned on the X-axis on the 2026 game field.
    //------------------------------------------------------------------
    FieldOffsetValues();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~FieldOffsetValues() = default;

    /// @brief Singleton instance pointer
    static FieldOffsetValues *m_instance;

    /// @brief X-coordinate offset from the blue alliance depot neutral side (meters)
    units::length::meter_t m_blueDepotX;

    /// @brief X-coordinate offset from the red alliance depot neutral side (meters)
    units::length::meter_t m_redDepotX;

    /// @brief X-coordinate offset from the blue alliance hub center (meters)
    units::length::meter_t m_blueHubX;

    /// @brief X-coordinate offset from the red alliance hub center (meters)
    units::length::meter_t m_redHubX;

    /// @brief X-coordinate offset from the blue alliance outpost (meters)
    units::length::meter_t m_blueOutpostX;

    /// @brief X-coordinate offset from the red alliance outpost (meters)
    units::length::meter_t m_redOutpostX;

    /// @brief X-coordinate offset from the bump edge (meters)
    units::length::meter_t m_redAllianceBumpEdgeX;
    units::length::meter_t m_redNeutralBumpEdgeX;
    units::length::meter_t m_blueAllianceBumpEdgeX;
    units::length::meter_t m_blueNeutralBumpEdgeX;

    static constexpr units::length::meter_t HUB_OFFSET = 2.0_m;
    static constexpr units::length::meter_t BUMP_OFFSET = 1.1_m;
};
