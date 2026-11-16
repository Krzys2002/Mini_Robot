#pragma once
#include "HomographyFixtures.hpp"

namespace micro_robot::test {
// Independent pinhole construction: camera (0.75,-1,0.5) m looks horizontally
// along world +Y, image +u follows +X, image +v points toward the table below it.
// Table: 1.5 x 2.4 m. Near/far depth is 1.0/3.4 m, so near/far scale ratio is 3.4.
inline ImageCorner obliqueImage(WorldPoint2D point, double heightAboveTableMeters = 0)
{
    const double depth = 1 + point.y.value;
    return {500 + 400 * (point.x.value - 0.75) / depth,
            300 + 410 * (0.5 - heightAboveTableMeters) / depth};
}
inline WorkspaceHomography obliqueHomography()
{
    return {{0.5125,0.75,-481.25, 0,-1,505, 0,1,-300}};
}
inline WorkspaceHomographyInput obliqueWorkspaceInput()
{
    auto input = homographyInput();
    input.profile.bounds = WorkspaceBounds{Meters{1.5},Meters{2.4}};
    input.profile.intrinsics.cameraMatrix = {400,0,500, 0,410,300, 0,0,1};
    input.profile.intrinsics.distortion = {0,0,0,0,0};
    input.fitPoints.clear(); input.checkPoints.clear();
    for (const auto xy : std::array<std::array<double,2>,4>{{{0,0},{1.5,0},{1.5,2.4},{0,2.4}}}) {
        const WorldPoint2D world{Meters{xy[0]},Meters{xy[1]}};
        input.fitPoints.push_back({obliqueImage(world),world});
    }
    for (const auto xy : std::array<std::array<double,2>,5>{{{0.2,0},{1.1,2.4},{0,1.2},{1.5,0.6},{0.7,1.1}}}) {
        const WorldPoint2D world{Meters{xy[0]},Meters{xy[1]}};
        input.checkPoints.push_back({obliqueImage(world),world});
    }
    return input;
}
} // namespace micro_robot::test
