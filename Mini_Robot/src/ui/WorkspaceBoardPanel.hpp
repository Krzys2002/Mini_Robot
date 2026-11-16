#pragma once
#include <limits>
#include "app/WorkspaceCalibrationController.hpp"
#include <QGroupBox>
#include <functional>
class QDoubleSpinBox; class QSpinBox; class QComboBox; class QCheckBox; class QTableWidget;
class QLineEdit; class QLabel; class QPushButton;
namespace micro_robot {
inline constexpr double WorkspaceBoardAnchorPickRadiusPixels=12;
inline constexpr double WorkspaceBoardZoomMarginPixels=16;
class WorkspaceBoardPanel final : public QGroupBox {
public:
    WorkspaceBoardPanel(WorkspaceCalibrationController& controller,
                        std::function<std::optional<std::uint64_t>()> selectedReference,QWidget* parent=nullptr);
private:
    void refresh();
    void edited();
    void review();
    WorkspaceCalibrationController& controller_;
    std::function<std::optional<std::uint64_t>()> selectedReference_;
    QWidget* settings_{};
    QDoubleSpinBox *x_{},*y_{},*angle_{},*spacing_{};
    QSpinBox* anchor_{};
    QComboBox* rowDirection_{};
    QCheckBox *reverseColumns_{},*reverseRows_{},*relative_{},*identity_{},*check_{};
    QLineEdit *note_{}, *surface_{}, *boardThickness_{}, *backingThickness_{}, *elevationMethod_{};
    QTableWidget* points_{};
    QLabel* details_{};
    QPushButton *freeze_{},*detect_{},*review_{},*import_{},*reference_{};
    std::optional<std::uint64_t> relativeId_;
    std::string frameId_;
    std::uint64_t shownRevision_{std::numeric_limits<std::uint64_t>::max()};
    static constexpr int CoordinateDecimals=9;
    static constexpr double MaximumAnchorMeters=1000;
    static constexpr double MaximumAngleDegrees=360;
    static constexpr double MinimumSpacingMeters=0.000001;
    static constexpr double MaximumSpacingMeters=1;
};
} // namespace micro_robot
