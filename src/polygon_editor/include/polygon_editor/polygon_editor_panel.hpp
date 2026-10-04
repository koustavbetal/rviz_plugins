#pragma once

#include <rviz_common/panel.hpp>

class QPushButton;
class QLabel;
class QDoubleSpinBox;
class QCheckBox;

namespace polygon_editor
{

class PolygonEditorPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PolygonEditorPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void onNewClicked();
  void onUndoClicked();
  void onSendClicked();
  void refreshStatus();
  void onRecordClicked();
  

private:
  void ensureAuxDisplays();

  QPushButton * new_button_;
  QPushButton * undo_button_;
  QPushButton * send_button_;
  QLabel * status_label_;
  QDoubleSpinBox * headland_width_spin_;
  QDoubleSpinBox * swath_angle_spin_;
  QCheckBox * use_set_angle_check_;
  QPushButton * record_button_;
};

}  // namespace polygon_editor