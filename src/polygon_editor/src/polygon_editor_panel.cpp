#include "polygon_editor/polygon_editor_panel.hpp"
#include "polygon_editor/polygon_store.hpp"

#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFormLayout>

#include <pluginlib/class_loader.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/property.hpp>

namespace polygon_editor
{

PolygonEditorPanel::PolygonEditorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  new_button_ = new QPushButton("New Polygon");
  load_button_ = new QPushButton("Load Polygon");
  undo_button_ = new QPushButton("Undo Last Point");
  send_button_ = new QPushButton("Send to Coverage Server");
  status_label_ = new QLabel("Vertices: 0");
  record_button_ = new QPushButton("Record Robot Pose Here");

  headland_width_spin_ = new QDoubleSpinBox;
  headland_width_spin_->setRange(0.0, 5.0);
  headland_width_spin_->setSingleStep(0.05);
  headland_width_spin_->setValue(0.3);

  swath_angle_spin_ = new QDoubleSpinBox;
  swath_angle_spin_->setRange(0.0, 360.0);
  swath_angle_spin_->setSingleStep(1.0);
  swath_angle_spin_->setValue(0.0);

  use_set_angle_check_ = new QCheckBox("Use fixed swath angle");
  use_set_angle_check_->setChecked(true);

  auto * layout = new QVBoxLayout;

  auto * form = new QFormLayout;
  form->addRow("Headland width (m)", headland_width_spin_);
  form->addRow("Swath angle (deg)", swath_angle_spin_);
  form->addRow(use_set_angle_check_);
  layout->addLayout(form);

  layout->addWidget(new_button_);
  layout->addWidget(load_button_);
  layout->addWidget(undo_button_);
  layout->addWidget(status_label_);
  layout->addWidget(send_button_);
  layout->addWidget(record_button_);
  setLayout(layout);

  connect(new_button_, &QPushButton::clicked, this, &PolygonEditorPanel::onNewClicked);
  connect(load_button_, &QPushButton::clicked, this, &PolygonEditorPanel::onLoadClicked);
  connect(undo_button_, &QPushButton::clicked, this, &PolygonEditorPanel::onUndoClicked);
  connect(send_button_, &QPushButton::clicked, this, &PolygonEditorPanel::onSendClicked);
  connect(record_button_, &QPushButton::clicked, this, &PolygonEditorPanel::onRecordClicked);

  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &PolygonEditorPanel::refreshStatus);
  timer->start(300);
}

void PolygonEditorPanel::onInitialize()
{
  PolygonStore::instance().ensureInitialized();
  ensureAuxDisplays();
}
void PolygonEditorPanel::ensureAuxDisplays()
{
  // Public, supported way to instantiate a Display plugin from outside
  // rviz_common — DisplayFactory (which does the same thing internally)
  // lives under a private header and isn't installed for plugin authors.
  static pluginlib::ClassLoader<rviz_common::Display> display_loader(
    "rviz_common", "rviz_common::Display");

  auto * root = getDisplayContext()->getRootDisplayGroup();

  bool has_markers = false;
  bool has_line = false;
  for (int i = 0; i < root->numDisplays(); ++i) {
    rviz_common::Display * d = root->getDisplayAt(i);
    if (d->getName() == "Polygon Vertices") { has_markers = true; }
    if (d->getName() == "Polygon Outline") { has_line = true; }
  }

  if (!has_markers) {
    try {
      rviz_common::Display * im_display =
        display_loader.createUnmanagedInstance("rviz_default_plugins/InteractiveMarkers");
      im_display->initialize(getDisplayContext());
      im_display->setName("Polygon Vertices");
      im_display->subProp("Update Topic")->setValue("polygon_editor_markers/update");
      root->addChild(im_display);
      im_display->setEnabled(true);
    } catch (const pluginlib::PluginlibException &) {
      // Display type not found — leave it for manual Add Display.
    }
  }

  if (!has_line) {
    try {
      rviz_common::Display * line_display =
        display_loader.createUnmanagedInstance("rviz_default_plugins/Marker");
      line_display->initialize(getDisplayContext());
      line_display->setName("Polygon Outline");
      line_display->subProp("Topic")->setValue("coverage_polygon_outline");
      root->addChild(line_display);
      line_display->setEnabled(true);
    } catch (const pluginlib::PluginlibException &) {
    }
  }
}

void PolygonEditorPanel::onNewClicked()
{
  PolygonStore::instance().clear();
  setStatus("New polygon started.");
}

void PolygonEditorPanel::onLoadClicked()
{
  double swath_angle_deg = 0.0;
  if (!PolygonStore::instance().loadSavedPolygon(swath_angle_deg)) {
    setStatus("Load failed — see ROS log.");
    return;
  }

  swath_angle_spin_->setValue(swath_angle_deg);
  setStatus("Polygon loaded.");
}

void PolygonEditorPanel::onUndoClicked()
{
  PolygonStore::instance().undoLast();
  setStatus("Removed last point.");
}

void PolygonEditorPanel::onSendClicked()
{
  if (PolygonStore::instance().vertexCount() < 3) {
    setStatus("Need >= 3 vertices.");
    return;
  }

  bool ok = PolygonStore::instance().sendToCoverageServer(
    headland_width_spin_->value(),
    swath_angle_spin_->value(),
    use_set_angle_check_->isChecked());
  setStatus(ok ? "Sent." : "Send failed — see ROS log.");
}
void PolygonEditorPanel::refreshStatus()
{
  const QString count =
    QString("Vertices: %1").arg(PolygonStore::instance().vertexCount());
  status_label_->setText(status_message_.isEmpty() ? count : status_message_ + " | " + count);
}

void PolygonEditorPanel::onRecordClicked()
{
  bool ok = PolygonStore::instance().recordCurrentRobotPose();
  setStatus(ok ? "Point recorded." : "TF lookup failed — check tree.");
}

void PolygonEditorPanel::setStatus(const QString & message)
{
  status_message_ = message;
  refreshStatus();
}
}  // namespace polygon_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(polygon_editor::PolygonEditorPanel, rviz_common::Panel)