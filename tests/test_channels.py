"""ChannelLayout must reproduce the legacy servo_const masks exactly."""

import pytest

from servo_aligner.channels import ChannelLayout

CHANNELS = ["A_x", "A_y", "A_xdot", "A_ydot", "B_x", "B_y", "B_xdot", "B_ydot"]

# faithful to legacy servo_const.py, including the mirrored B-path quirk
GROUPS = {
    "A_X_XDOT": ["A_x", "A_xdot"],
    "A_Y_YDOT": ["A_y", "A_ydot"],
    "A_X_Y": ["A_x", "A_y"],
    "A_XDOT_YDOT": ["A_xdot", "A_ydot"],
    "A_POS_ALL": ["A_x", "A_y", "A_xdot", "A_ydot"],
    "B_X_XDOT": ["B_x", "B_xdot"],
    "B_Y_YDOT": ["B_y", "B_ydot"],
    "B_X_Y": ["B_xdot", "B_ydot"],
    "B_XDOT_YDOT": ["B_x", "B_y"],
    "B_POS_ALL": ["B_x", "B_y", "B_xdot", "B_ydot"],
    "ALL": "*",
}


@pytest.fixture
def layout():
    return ChannelLayout(CHANNELS, GROUPS)


def test_masks_match_legacy_constants(layout, golden):
    legacy = golden["legacy_masks"]
    for legacy_name, legacy_mask in legacy.items():
        group_name = legacy_name[: -len("_MASK")]
        if group_name == "POS_ALL":
            group_name = "ALL"
        assert list(layout.group(group_name).mask) == legacy_mask, group_name


def test_group_name_travels_with_mask(layout):
    g = layout.group("A_X_XDOT")
    assert g.name == "A_X_XDOT"
    assert g.n == 2
    assert g.channels == ("A_x", "A_xdot")


def test_member_order_normalized_to_channel_index(layout):
    # B_XDOT_YDOT lists B_x/B_y; ordering must follow channel index
    g = layout.group("B_XDOT_YDOT")
    assert g.channels == ("B_x", "B_y")
    assert g.mask == (0, 0, 0, 0, 1, 1, 0, 0)


def test_all_group(layout):
    assert layout.all.mask == (1,) * 8
    assert layout.group("ALL").mask == (1,) * 8


def test_single(layout):
    assert layout.single("B_y").mask == (0, 0, 0, 0, 0, 1, 0, 0)
    assert layout.single(2).channels == ("A_xdot",)


def test_unknown_lookups_raise(layout):
    with pytest.raises(KeyError):
        layout.group("NOPE")
    with pytest.raises(KeyError):
        layout.mask("ghost_channel")


def test_n_channel_generality():
    small = ChannelLayout(["x", "y"], {"XY": ["x", "y"]})
    assert small.group("XY").mask == (1, 1)
    assert small.n == 2
