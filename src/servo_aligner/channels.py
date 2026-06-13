"""Named channel groups and masks, derived from configuration.

Replaces the legacy hardcoded 8-element ``*_MASK`` constants in
``servo_const.py``. A :class:`ChannelGroup` carries its name together with
its 0/1 mask, so code no longer identifies masks by equality comparison
(the old ``posmask2str`` hack) and layouts of any channel count work.

Note: the *reduced*-vector ordering used throughout the optimization code is
by **channel index**, not by the order channels are listed in a group.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple, Union


@dataclass(frozen=True)
class ChannelGroup:
    """A named subset of channels with its 0/1 selection mask."""

    name: str
    channels: Tuple[str, ...]
    mask: Tuple[int, ...]

    @property
    def n(self) -> int:
        """Number of selected channels (the reduced-vector length)."""
        return int(sum(self.mask))

    def __str__(self) -> str:
        return self.name


class ChannelLayout:
    """The full channel list of a device plus its named groups.

    Args:
        channel_names: Ordered channel names; the index in this sequence is
            the channel index everywhere (masks, vectors, servo wiring).
        groups: Mapping of group name to a list of member channel names, or
            the string ``"*"`` for all channels.
    """

    def __init__(
        self,
        channel_names: Sequence[str],
        groups: Mapping[str, Union[Sequence[str], str]] = (),
    ):
        names = tuple(channel_names)
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate channel names: {names}")
        self.names: Tuple[str, ...] = names
        self._groups: dict = {}
        for group_name, members in dict(groups).items():
            if members == "*":
                members = names
            # normalize member order to channel-index order, matching the
            # reduced-vector convention used by the mask helpers
            members = tuple(sorted(members, key=self.index))
            self._groups[group_name] = ChannelGroup(
                name=group_name,
                channels=members,
                mask=self.mask(*members),
            )

    @property
    def n(self) -> int:
        return len(self.names)

    def __len__(self) -> int:
        return len(self.names)

    @property
    def group_names(self) -> Tuple[str, ...]:
        return tuple(self._groups)

    def index(self, channel: str) -> int:
        try:
            return self.names.index(channel)
        except ValueError:
            raise KeyError(
                f"unknown channel {channel!r}; available: {list(self.names)}"
            ) from None

    def mask(self, *channels: str) -> Tuple[int, ...]:
        """Build a 0/1 mask selecting the given channels (ad-hoc groups)."""
        mask = [0] * len(self.names)
        for ch in channels:
            mask[self.index(ch)] = 1
        return tuple(mask)

    def group(self, name: str) -> ChannelGroup:
        try:
            return self._groups[name]
        except KeyError:
            raise KeyError(
                f"unknown channel group {name!r}; available: {list(self._groups)}"
            ) from None

    def __contains__(self, name: str) -> bool:
        return name in self._groups

    def single(self, channel: Union[str, int]) -> ChannelGroup:
        """A one-channel group, by channel name or index."""
        if isinstance(channel, int):
            channel = self.names[channel]
        return ChannelGroup(
            name=channel, channels=(channel,), mask=self.mask(channel)
        )

    @property
    def all(self) -> ChannelGroup:
        """The group of every channel (replaces the legacy POS_ALL_MASK)."""
        return ChannelGroup(
            name="ALL", channels=self.names, mask=(1,) * len(self.names)
        )
