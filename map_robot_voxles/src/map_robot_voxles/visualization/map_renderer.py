"""Point-map rendering that can be embedded in an existing Plotter."""

from .scene import ActorLayer


class MapRenderer:
    def __init__(self, plotter, prefix="map"):
        self.layer = ActorLayer(plotter, prefix)

    def render(self, point_map, stride=1, show_surface=True):
        stride = max(1, int(stride))
        self.layer.points(
            "surface", point_map.points[::stride], color="lightgray",
            point_size=3, opacity=0.18,
        )
        self.layer.points(
            "landing", point_map.landing_points[::stride], color="black",
            point_size=5, opacity=0.22,
        )
        self.layer.visible("surface", show_surface)
        return self.layer

    def set_visible(self, visible):
        for name in self.layer.names:
            actor = self.layer.plotter.actors.get(name)
            if actor is not None:
                actor.SetVisibility(bool(visible))

    def remove(self):
        self.layer.clear()

