#include "polygon_editor/polygon_edit_tool.hpp"
#include "polygon_editor/polygon_store.hpp"

#include <Ogre.h>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>

namespace polygon_editor
{

PolygonEditTool::PolygonEditTool()
{
  shortcut_key_ = 'p';
}

void PolygonEditTool::onInitialize()
{
  setName("Add Polygon");
  // TODO: point this at an icon of your choosing, e.g.:
  // setIcon(QIcon(":/images/polygon.png"));
}

void PolygonEditTool::activate()
{
  PolygonStore::instance().ensureInitialized();
}

void PolygonEditTool::deactivate()
{
}

int PolygonEditTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!event.leftDown()) {
    return Render;
  }

  // GPU-based pick against whatever is actually rendered at this pixel
  // (the map plane, in your case). This is what rviz's own built-in tools
  // (e.g. FocusTool, InteractionTool) use — it sidesteps needing direct
  // Ogre::Camera/Viewport access, which isn't part of the stable
  // rviz_common::RenderPanel API on the Ogre-Next render backend (Jazzy+).
  Ogre::Vector3 hit;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, hit);
  if (!success) {
    // Nothing rendered under the cursor at that pixel (e.g. clicked off
    // the edge of the loaded map) — ignore the click.
    return Render;
  }

  geometry_msgs::msg::Point p;
  p.x = hit.x;
  p.y = hit.y;
  p.z = 0.0;

  PolygonStore::instance().addVertex(p);

  return Render;
}

}  // namespace polygon_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(polygon_editor::PolygonEditTool, rviz_common::Tool)