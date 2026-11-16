#include "calibration/WorkspaceAcquisition.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include "HomographyFixtures.hpp"
#include <gtest/gtest.h>
#include <numbers>
using namespace micro_robot;
using namespace micro_robot::test;
namespace {
WorkspaceBoardPlacement board()
{
    WorkspaceBoardPlacement p; p.id=1; p.columnAngle=Radians{0}; p.rowDirection=1; p.anchor={Meters{0.2},Meters{0.2}};
    p.frameSequence=12; p.capturedTicksNanoseconds=12345; p.note="measured placement";
    for (int row=0;row<WorkspaceBoardTarget.rows;++row) for (int col=0;col<WorkspaceBoardTarget.columns;++col) {
        p.detected.push_back({100.0+col*20,100.0+row*20}); p.included.push_back(true);
    }
    return p;
}
}
TEST(WorkspaceAcquisitionTest, ExplicitAnchorOrientationAndReversalsDetermineEveryPair)
{
    auto p=board(); p.anchorIndex=13; p.columnAngle=Radians{std::numbers::pi/2}; p.rowDirection=-1;
    auto result=generateWorkspaceBoard(p);
    ASSERT_TRUE(result.error.empty())<<result.error; ASSERT_EQ(result.points.size(),54);
    EXPECT_DOUBLE_EQ(result.points[13].world.x.value,0.2);
    EXPECT_DOUBLE_EQ(result.points[13].world.y.value,0.2);
    EXPECT_NEAR(result.points[14].world.y.value,0.225,1e-14);
    EXPECT_NEAR(result.points[22].world.x.value,0.225,1e-14);
    const auto original=result.points[0].image;
    p.reverseColumns=true; p.reverseRows=true;
    result=generateWorkspaceBoard(p);
    ASSERT_TRUE(result.error.empty());
    EXPECT_DOUBLE_EQ(result.points[53].image.u,original.u);
    EXPECT_DOUBLE_EQ(result.points[53].image.v,original.v);
    p.rowDirection=0; EXPECT_FALSE(generateWorkspaceBoard(p).error.empty());
    p=board(); p.detected.pop_back(); EXPECT_FALSE(generateWorkspaceBoard(p).error.empty());
    p=board(); p.anchorIndex=54; EXPECT_FALSE(generateWorkspaceBoard(p).error.empty());
    p=board(); p.detected[1]=p.detected[0]; EXPECT_FALSE(generateWorkspaceBoard(p).error.empty());
    p=board(); for (auto& pixel:p.detected) pixel.v=pixel.u;
    EXPECT_FALSE(generateWorkspaceBoard(p).error.empty());
    p=board(); p.columnAngle=Radians{std::numeric_limits<double>::quiet_NaN()};
    EXPECT_FALSE(generateWorkspaceBoard(p).error.empty());
}
TEST(WorkspaceAcquisitionTest, Inclusive1525LocationLatticeUsesReviewedOverlappingPlacements)
{
    auto profile=homographyInput().profile;
    profile.bounds=WorkspaceBounds{Meters{1.5},Meters{0.6}};
    profile.acquisitionId="lattice-record";
    profile.boundary=WorkspaceBoundary{{{50,50},{800,50},{800,530},{50,530}}};
    profile.homography=WorkspaceHomography{{0.002,0,-0.1,0,0.00125,-0.0625,0,0,1}};
    WorkspaceAcquisitionRecord record{profile.acquisitionId,profile,{},{}};
    std::vector<WorkspaceCorrespondence> existing;
    const auto worldCorners=workspaceCorners(profile.bounds);
    std::uint64_t nextId=1;
    for (std::size_t i=0;i<WorkspaceCornerCount;++i) {
        WorkspaceCorrespondence pair{(*profile.boundary)[i],worldCorners[i]}; existing.push_back(pair);
        record.references.push_back({nextId++,pair,false,true,{}});
    }
    for (int y : {0,5,10,15,19}) for (int x : {0,8,16,24,32,40,48,52}) {
        auto p=board(); p.id=record.placements.size()+1;
        p.anchor={Meters{x*0.025},Meters{y*0.025}}; p.detected.clear();
        for (int row=0;row<WorkspaceBoardTarget.rows;++row) for (int col=0;col<WorkspaceBoardTarget.columns;++col)
            p.detected.push_back({50+500*(x+col)*0.025,50+800*(y+row)*0.025});
        auto generated=generateWorkspaceBoard(p,profile.bounds);
        ASSERT_TRUE(generated.error.empty());
        auto assessment=assessWorkspaceReferences(profile,existing,generated.points);
        ASSERT_TRUE(assessment.error.empty())<<assessment.error;
        p.reviewRejections=assessment.candidates;
        for (std::size_t i=0;i<generated.points.size();++i) {
            p.included[i]=assessment.candidates[i]==WorkspaceReferenceRejection::None;
            if (p.included[i]) {
                record.references.push_back({nextId++,generated.points[i],false,false,WorkspaceBoardSource{p.id,i}});
                existing.push_back(generated.points[i]);
            }
        }
        record.placements.push_back(std::move(p));
    }
    ASSERT_EQ(record.placements.size(),40);
    ASSERT_EQ(existing.size(),1525);
    ASSERT_TRUE(validateWorkspaceAcquisition(record).empty())<<validateWorkspaceAcquisition(record);
    record.placements.front().surfaceElevationMeters=0.004;
    record.placements.front().boardThicknessMeters=0.003;
    record.placements.front().backingThicknessMeters=0.001;
    record.placements.front().elevationMethod="caliper survey from tabletop";
    const auto text=serializeWorkspaceAcquisition(record);
    EXPECT_LT(text.size(),MaximumWorkspaceAcquisitionBytes);
    auto restored=parseWorkspaceAcquisition(text);
    ASSERT_TRUE(restored.record)<<restored.error;
    EXPECT_EQ(restored.record->references.size(),1525);
    EXPECT_EQ(restored.record->placements.size(),40);
    EXPECT_EQ(serializeWorkspaceAcquisition(*restored.record),text);
    EXPECT_EQ(restored.record->placements.front().surfaceElevationMeters,0.004);
    auto legacy=text;
    legacy.replace(legacy.find("acquisition 2"),std::string("acquisition 2").size(),"acquisition 1");
    for (auto at=legacy.find(" elevation "); at!=std::string::npos; at=legacy.find(" elevation "))
        legacy.erase(at,legacy.find('\n',at)-at);
    const auto old=parseWorkspaceAcquisition(legacy);
    ASSERT_TRUE(old.record)<<old.error;
    EXPECT_FALSE(old.record->placements.front().surfaceElevationMeters);
    EXPECT_FALSE(old.record->placements.front().boardThicknessMeters);
    auto mismatch=text;
    mismatch.replace(mismatch.find("acquisition 2"),std::string("acquisition 2").size(),"acquisition 1");
    EXPECT_FALSE(parseWorkspaceAcquisition(mismatch).record);
    record.placements.front().elevationMethod.clear();
    EXPECT_FALSE(validateWorkspaceAcquisition(record).empty());
    restored.record->references[4].point.world.x.value+=0.001; // Manual correction retains original placement/detection.
    EXPECT_TRUE(validateWorkspaceAcquisition(*restored.record).empty());
    EXPECT_EQ(restored.record->placements[0].detected.size(),54);
    EXPECT_FALSE(parseWorkspaceAcquisition(text+"garbage").record);
    restored.record->placements[0].included[0]=true; // Inconsistent duplicate-exclusion provenance.
    EXPECT_FALSE(validateWorkspaceAcquisition(*restored.record).empty());
}
TEST(WorkspaceAcquisitionTest, SpatialDuplicateAssessmentChecksNeighbourCellsAndBothDomains)
{
    auto profile=homographyInput().profile;
    const auto p=homographyInput().fitPoints.front();
    auto near=p; near.image.u+=0.5e-8*profile.intrinsics.width;
    near.world={Meters{0.2},Meters{0.2}};
    auto worldDuplicate=p; worldDuplicate.image={500,400};
    auto outside=p; outside.world.x=Meters{-0.01};
    auto result=assessWorkspaceReferences(profile,{p},{near,worldDuplicate,outside});
    ASSERT_TRUE(result.error.empty());
    EXPECT_EQ(result.candidates[0],WorkspaceReferenceRejection::Duplicate);
    EXPECT_EQ(result.candidates[1],WorkspaceReferenceRejection::Duplicate);
    EXPECT_EQ(result.candidates[2],WorkspaceReferenceRejection::OutsideDomain);
}

TEST(WorkspaceAcquisitionTest, VersionTwoProfilesKeepTheirFrameWithoutInventedProvenance)
{
    auto p=homographyInput().profile; p.homography=knownHomography();
    p.bounds=WorkspaceBounds{Meters{1},Meters{0.6},{Meters{-0.5},Meters{-0.3}}};
    auto text=serializeWorkspaceProfile(p);
    text.replace(text.find("workspace 3"),std::string("workspace 3").size(),"workspace 2");
    const auto begin=text.find("acquisition "); text.erase(begin,text.find('\n',begin)-begin+1);
    const auto migrated=parseWorkspaceProfile(text); ASSERT_TRUE(migrated.profile)<<migrated.error;
    EXPECT_TRUE(migrated.profile->bounds==p.bounds);
    EXPECT_EQ(migrated.profile->homography->imageToWorld,p.homography->imageToWorld);
    EXPECT_TRUE(migrated.profile->acquisitionId.empty());
    EXPECT_EQ(migrated.profile->version,WorkspaceCalibrationProfileVersion);
}
TEST(WorkspaceAcquisitionTest, RelativeAnchorSnapshotAndBoundsRoundoffAreExplicit)
{
    auto p=board(); p.anchor={Meters{0},Meters{0.47500000000000003}};
    const WorkspaceBounds bounds{Meters{1.5},Meters{0.6}};
    const auto generated=generateWorkspaceBoard(p,bounds); ASSERT_TRUE(generated.error.empty());
    EXPECT_DOUBLE_EQ(generated.points.back().world.y.value,0.6);
    p.anchor.y.value+=0.0001;
    const auto outside=generateWorkspaceBoard(p,bounds); ASSERT_TRUE(outside.error.empty());
    EXPECT_FALSE(bounds.contains(outside.points.back().world)); // No physical clamping.
    EXPECT_FALSE(parseWorkspaceAcquisition(std::string(MaximumWorkspaceAcquisitionBytes+1,'x')).record);
    EXPECT_FALSE(parseWorkspaceAcquisition("micro_robot_workspace_acquisition 99").record);
}

TEST(WorkspaceAcquisitionTest, RejectsMalformedIdentifiersCountsAndMismatchedBindings)
{
    auto input=homographyInput(); auto p=input.profile; p.acquisitionId="record"; p.homography=knownHomography();
    WorkspaceAcquisitionRecord record{p.acquisitionId,p,{},{}};
    std::uint64_t id=1; for (const auto& pair:input.fitPoints) record.references.push_back({id++,pair,false,false,{}});
    const auto text=serializeWorkspaceAcquisition(record);
    auto bad=text; bad.replace(bad.find("reference 1 "),std::string("reference 1 ").size(),"reference -2 ");
    EXPECT_FALSE(parseWorkspaceAcquisition(bad).record);
    bad=text; bad.replace(bad.find("references 4"),std::string("references 4").size(),"references 999999999999999999999");
    EXPECT_FALSE(parseWorkspaceAcquisition(bad).record);
    record.id="different"; EXPECT_FALSE(validateWorkspaceAcquisition(record).empty());
    record.id=p.acquisitionId; record.placements.resize(MaximumWorkspacePlacements+1);
    EXPECT_FALSE(validateWorkspaceAcquisition(record).empty());
}
