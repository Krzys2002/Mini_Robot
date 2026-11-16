#pragma once
#include "calibration/TabletopHomography.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include "calibration/WorkspaceAcquisition.hpp"
#include "infrastructure/logging/Logger.hpp"
#include <QObject>
#include <QString>
#include <QThread>
#include <memory>

namespace micro_robot {
// GUI-thread application service. Undistortion, estimation and disk I/O share
// one serialized worker. Input/configuration changes invalidate late results.
class WorkspaceCalibrationController final : public QObject {
    Q_OBJECT
public:
    WorkspaceCalibrationController(std::unique_ptr<IWorkspaceHomographySolver> solver,
                                   std::unique_ptr<ICalibrationUndistorter> undistorter,
                                   std::unique_ptr<IWorkspaceProfileStore> store, LogChannel log,
                                   QObject* parent = nullptr);
    ~WorkspaceCalibrationController() override;
    void setInput(std::optional<IntrinsicCalibration> intrinsics, bool persistent);
    void receiveFrame(FramePtr frame);
    void configure(QString workspaceId, double widthMeters, double heightMeters,
                   double originFromBaseXMeters = 0, double originFromBaseYMeters = 0);
    void selectCorner(ImageCorner point, std::optional<std::size_t> replace = {});
    void undoCorner();
    void clearCorners();
    void confirmBoundary();
    void setOriginFromImage(ImageCorner point);
    [[nodiscard]] const auto& frameId() const { return frameId_; }
    [[nodiscard]] const auto& corners() const { return corners_; }
    [[nodiscard]] std::uint64_t referencesRevision() const { return referencesRevision_; }
    [[nodiscard]] bool boundaryConfirmed() const { return boundaryConfirmed_; }
    [[nodiscard]] bool canAddReferences() const { return boundaryConfirmed_ || (draft_ && draft_->homography); }
    void setAcquisitionDetector(std::unique_ptr<ICalibrationDetector> detector);
    void editReference(std::size_t index, WorkspaceCorrespondence point, bool check);
    void appendReference(std::optional<ImageCorner> point = {});
    void removeReference(std::size_t index);
    [[nodiscard]] const auto& references() const { return references_; }
    [[nodiscard]] const auto& placements() const { return placements_; }
    [[nodiscard]] const auto& boardReview() const { return boardReview_; }
    [[nodiscard]] const auto& boardPoints() const { return boardPoints_; }
    [[nodiscard]] const auto& boardAssessment() const { return boardAssessment_; }
    [[nodiscard]] bool canDetectBoard() const { return !busy_ && !shuttingDown_ && frozen_ && detector_ && boundaryConfirmed_; }
    [[nodiscard]] bool canImportBoard() const { return !busy_ && !shuttingDown_ && frozen_ && boardReview_ && boundaryConfirmed_; }
    void discardBoardReview();
    [[nodiscard]] std::uint64_t boardRevision() const { return boardRevision_; }
    [[nodiscard]] const auto& detectedBoard() const { return detectedBoard_; }
    [[nodiscard]] bool canReviewBoard() const { return !busy_ && !shuttingDown_ && frozen_ && detectedBoard_ && boundaryConfirmed_; }
    void reviewBoardPlacement(WorkspaceBoardPlacement settings);
    void setBoardPointIncluded(std::size_t index, bool included);
    void importBoardPlacement(bool check, bool physicalIdentityConfirmed);
    void setPoints(std::vector<WorkspaceCorrespondence> fit, std::vector<WorkspaceCorrespondence> check);

    [[nodiscard]] const auto& activeAcquisition() const { return activeAcquisition_; }
    [[nodiscard]] const auto& draftAcquisition() const { return draftAcquisition_; }
    [[nodiscard]] const std::optional<WorkspaceCalibrationProfile>& profile() const { return profile_; }

    [[nodiscard]] const std::optional<WorkspaceCalibrationProfile>& draftProfile() const { return draft_; }

    [[nodiscard]] const auto& fitPoints() const { return fit_; }

    [[nodiscard]] const auto& checkPoints() const { return check_; }

    [[nodiscard]] QString workspaceId() const { return workspaceId_; }

    [[nodiscard]] std::optional<WorkspaceBounds> draftBounds() const { return bounds_; }

    [[nodiscard]] bool canVerify() const
    {
        return !shuttingDown_ && !busy_ && frozen_ && draft_ && !check_.empty();
    }

    [[nodiscard]] bool canActivate() const
    {
        return !shuttingDown_ && !busy_ && draft_ && draft_->homography
            && (draft_->acquisitionId.empty() || (draftAcquisition_ && draftAcquisition_->id==draft_->acquisitionId));
    }

    [[nodiscard]] FramePtr frozenFrame() const { return frozen_; }

    [[nodiscard]] QString message() const { return message_; }

    [[nodiscard]] bool busy() const { return busy_; }

    [[nodiscard]] bool canFreeze() const { return !shuttingDown_ && !busy_ && intrinsics_ && latest_; }

    [[nodiscard]] bool canCalculate() const
    {
        return !shuttingDown_ && !busy_ && frozen_ && intrinsics_ && bounds_ && !workspaceId_.isEmpty() && boundaryConfirmed_;
    }

    [[nodiscard]] bool canSave() const
    {
        return !shuttingDown_ && !busy_ && persistent_ && draft_ && draft_->homography;
    }

    [[nodiscard]] bool canLoad() const
    {
        return !shuttingDown_ && !busy_ && persistent_ && intrinsics_ && !workspaceId_.isEmpty();
    }

    [[nodiscard]] bool isStopped() const { return worker_.isFinished(); }

public slots:
    void freeze();
    void freezeBoard();
    void detectBoard();
    void calculate();
    void verify();
    void activate();
    void save();
    void load();
    void shutdown();
signals:
    void changed();
    void profileChanged();
    void stopped();

private:
    void invalidate(bool clearFrame);
    void geometryChanged();
    void freezeFrame(bool preserveReferences);
    void synchronizeReferences();
    void referencesEdited();
    void clearBoardReview();
    [[nodiscard]] std::shared_ptr<WorkspaceAcquisitionRecord> acquisitionSnapshot(const WorkspaceCalibrationProfile& profile) const;
    void publish(QString message);
    std::shared_ptr<IWorkspaceHomographySolver> solver_;
    std::shared_ptr<ICalibrationUndistorter> undistorter_;
    std::shared_ptr<IWorkspaceProfileStore> store_;
    std::shared_ptr<ICalibrationDetector> detector_;
    std::vector<WorkspaceReference> references_;
    std::vector<WorkspaceBoardPlacement> placements_;
    std::optional<WorkspaceBoardPlacement> detectedBoard_, boardReview_;
    std::vector<WorkspaceCorrespondence> boardPoints_;
    std::vector<WorkspaceReferenceRejection> boardAssessment_;
    std::shared_ptr<const WorkspaceAcquisitionRecord> draftAcquisition_, activeAcquisition_;
    std::uint64_t nextReferenceId_{1}, nextPlacementId_{1}, boardRevision_{};
    LogChannel log_;
    QThread worker_;
    QObject* workerContext_{}; // Deleted on worker completion.
    std::optional<IntrinsicCalibration> intrinsics_;
    std::optional<WorkspaceBounds> bounds_;
    std::optional<WorkspaceCalibrationProfile> profile_; // Active, explicitly installed calibration.
    std::optional<WorkspaceCalibrationProfile> draft_;
    QString workspaceId_;
    std::string frameId_;
    std::vector<ImageCorner> corners_;
    bool boundaryConfirmed_{};
    FramePtr latest_, frozen_;
    std::vector<WorkspaceCorrespondence> fit_, check_;
    QString message_{
        QStringLiteral("Load or calculate compatible intrinsics, then freeze an undistorted frame.")};
    std::uint64_t generation_{}, referencesRevision_{};
    static constexpr auto MaximumFrozenFrameAge = std::chrono::seconds{2};
    bool persistent_{}, busy_{}, shuttingDown_{}, resolutionMismatch_{};
};
} // namespace micro_robot
