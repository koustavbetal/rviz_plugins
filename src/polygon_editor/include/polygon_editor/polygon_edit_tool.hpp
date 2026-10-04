#pragma once

#include <rviz_common/tool.hpp>

namespace Ogre { class SceneNode; }

namespace polygon_editor
{

/// RViz Tool: click in the 3D view to add a polygon vertex on the ground
/// plane (z = 0 in the "map" frame). Vertices are handed off to
/// PolygonStore, which owns the interactive markers you drag afterwards.
class PolygonEditTool : public rviz_common::Tool
{
public:
  PolygonEditTool();
  ~PolygonEditTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
};

} // namespace polygon_editor
