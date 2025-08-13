# layers/landmarks.py
import pandas as pd
import plotly.graph_objects as go

from carla_data_classes.enums.DataLandmarkType import DataLandmarkType
from carla_data_classes.static.DataWorld import DataWorld
from .base_layer import register, BaseLayer

# ----------------------------------------------------------------------
# Assign a distinct color to each landmark type (hex strings)
LANDMARK_COLOR_MAP = {
    DataLandmarkType.Danger.name: "#e41a1c",
    DataLandmarkType.LanesMerging.name: "#377eb8",
    DataLandmarkType.CautionPedestrian.name: "#4daf4a",
    DataLandmarkType.CautionBicycle.name: "#984ea3",
    DataLandmarkType.LevelCrossing.name: "#ff7f00",
    DataLandmarkType.StopSign.name: "#a65628",
    DataLandmarkType.YieldSign.name: "#f781bf",
    DataLandmarkType.MandatoryTurnDirection.name: "#999999",
    DataLandmarkType.MandatoryLeftRightDirection.name: "#66c2a5",
    DataLandmarkType.TwoChoiceTurnDirection.name: "#fc8d62",
    DataLandmarkType.Roundabout.name: "#8da0cb",
    DataLandmarkType.PassRightLeft.name: "#e78ac3",
    DataLandmarkType.AccessForbidden.name: "#a6d854",
    DataLandmarkType.AccessForbiddenMotorvehicles.name: "#ffd92f",
    DataLandmarkType.AccessForbiddenTrucks.name: "#e5c494",
    DataLandmarkType.AccessForbiddenBicycle.name: "#b3b3b3",
    DataLandmarkType.AccessForbiddenWeight.name: "#1b9e77",
    DataLandmarkType.AccessForbiddenWidth.name: "#d95f02",
    DataLandmarkType.AccessForbiddenHeight.name: "#7570b3",
    DataLandmarkType.AccessForbiddenWrongDirection.name: "#e7298a",
    DataLandmarkType.ForbiddenUTurn.name: "#66a61e",
    DataLandmarkType.MaximumSpeed.name: "#e6ab02",
    DataLandmarkType.ForbiddenOvertakingMotorvehicles.name: "#a6761d",
    DataLandmarkType.ForbiddenOvertakingTrucks.name: "#666666",
    DataLandmarkType.AbsoluteNoStop.name: "#1f78b4",
    DataLandmarkType.RestrictedStop.name: "#b2df8a",
    DataLandmarkType.HasWayNextIntersection.name: "#fb9a99",
    DataLandmarkType.PriorityWay.name: "#cab2d6",
    DataLandmarkType.PriorityWayEnd.name: "#6a3d9a",
    DataLandmarkType.CityBegin.name: "#ffff99",
    DataLandmarkType.CityEnd.name: "#b15928",
    DataLandmarkType.Highway.name: "#8b0000",
    DataLandmarkType.DeadEnd.name: "#00008b",
    DataLandmarkType.RecommendedSpeed.name: "#228b22",
    DataLandmarkType.RecommendedSpeedEnd.name: "#ff1493",
    DataLandmarkType.LightPost.name: "#222222",
}

# Fallback color
DEFAULT_LANDMARK_COLOR = "#888888"


@register("landmarks")
class LandmarkLayer(BaseLayer):
    slider_key = "landmarks"
    df_key = "landmarks"
    default_size = 6

    # ---------------------------------------------------------------- build df
    @classmethod
    def build_df(cls, data_world: DataWorld, tick):
        by_id = {}
        for road in data_world.get_all_roads():
            for lane in road.lanes:
                for lm in lane.landmarks or []:
                    rec = by_id.get(lm.id)
                    if rec is None:
                        rec = {
                            "x": lm.location.x,
                            "y": lm.location.y,
                            "id": lm.id,
                            "type": lm.type.name,
                            "orientation": lm.orientation.name,
                            "country": lm.country,
                            "text": lm.text,
                            "value": lm.value,
                            "sub_type": lm.sub_type,
                            "lane_pairs_set": set(),
                        }
                        by_id[lm.id] = rec
                    rid = getattr(lane, "road_id", getattr(road, "road_id", None))
                    rec["lane_pairs_set"].add((rid, lane.lane_id))

        rows = []
        for rec in by_id.values():
            pairs = sorted(rec.pop("lane_pairs_set"))
            lines = [f"&nbsp;&nbsp;&nbsp;&nbsp;(Road {r}, Lane {l})" for r, l in pairs]
            rec["lane_pairs_html"] = "<br>" + "<br>".join(lines) if lines else ""
            rows.append(rec)

        return pd.DataFrame(rows)

    # ---------------------------------------------------------------- traces
    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        traces = []
        for _, row in df.iterrows():
            # pick color based on landmark type
            color = LANDMARK_COLOR_MAP.get(row.type, DEFAULT_LANDMARK_COLOR)

            hover = (
                f"ID: {row.id}<br>"
                f"Type: {row.type}<br>"
                f"Orientation: {row.orientation}<br>"
                f"Country: {row.country}<br>"
                f"Text: {row.text}<br>"
                f"Value: {row.value}<br>"
                f"Sub-type: {row.sub_type}<br>"
                f"Lanes: {row.lane_pairs_html}<br>"
                f"X: {row.x:.2f} Y: {row.y:.2f}<extra></extra>"
            )

            traces.append(go.Scattergl(
                x=[row.x], y=[row.y],
                mode="markers",
                name=f"{row.type} - {row.id}",  # legend entry
                marker=dict(
                    size=self.size["landmarks"],
                    symbol="circle",
                    color=color,  # per-type color
                ),
                hovertemplate=hover,
                hoverlabel=dict(bgcolor=color),  # match hover bg to marker
                showlegend=True
            ))

        return traces
