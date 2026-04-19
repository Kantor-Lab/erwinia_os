#!/usr/bin/env python3
"""
Rewrite /robot_description mesh paths in an MCAP so Foxglove can resolve
meshes locally on this Mac. Input MCAP is unchanged; a new MCAP is produced
next to it with a _rewritten.mcap suffix.

Usage: python3 rewrite_urdf_mcap.py <input.mcap> [output.mcap]
"""

import sys
import re
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer

LOCAL_BASE = Path(
    "/Users/jackn/DevProjects/erwinia_os/gui/erwinia_os_foxglove/third_party_meshes/install"
)

# file:///home/appleseed_labs/erwinia_os_ws/install/<pkg>/share/<pkg>/...
ROBOT_PREFIX = re.compile(
    r"file:///home/appleseed_labs/erwinia_os_ws/install/"
)

# package://<pkg>/... → file:///LOCAL_BASE/<pkg>/share/<pkg>/...
PACKAGE_URI = re.compile(r"package://([A-Za-z0-9_\-]+)/")


def rewrite_urdf(text: str) -> str:
    text = ROBOT_PREFIX.sub(f"file://{LOCAL_BASE}/", text)

    def pkg_sub(match: re.Match) -> str:
        pkg = match.group(1)
        return f"file://{LOCAL_BASE}/{pkg}/share/{pkg}/"

    return PACKAGE_URI.sub(pkg_sub, text)


def rewrite_bytes(data: bytes) -> bytes:
    # /robot_description is std_msgs/String: 4-byte LE length + UTF-8 payload
    # + CDR header. Safer: decode latin-1 so byte offsets preserve, rewrite,
    # re-encode. We only touch ASCII URI strings so length shifts are handled
    # by rewriting the CDR length prefix if it changed.
    text = data.decode("latin-1")
    new_text = rewrite_urdf(text)
    if new_text == text:
        return data

    # The CDR wire format for std_msgs/String is:
    #   [4 bytes CDR header] [4-byte LE length including trailing NUL] [payload]
    # We need to patch the length field. Find it by locating the original
    # URDF start ("<?xml") and walking back 4 bytes.
    xml_idx = new_text.find("<?xml")
    if xml_idx < 4:
        # Unexpected layout — bail by returning original
        return data

    length_off = xml_idx - 4
    payload_len = len(new_text) - xml_idx
    import struct

    new_bytes = new_text.encode("latin-1")
    new_bytes = (
        new_bytes[:length_off]
        + struct.pack("<I", payload_len)
        + new_bytes[xml_idx:]
    )
    return new_bytes


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    in_path = Path(sys.argv[1])
    out_path = (
        Path(sys.argv[2])
        if len(sys.argv) > 2
        else in_path.with_name(in_path.stem + "_rewritten.mcap")
    )

    if not in_path.exists():
        print(f"Input MCAP not found: {in_path}")
        return 1

    print(f"Reading  {in_path}")
    print(f"Writing  {out_path}")

    total_msgs = 0
    rewritten_msgs = 0

    with open(in_path, "rb") as f_in, open(out_path, "wb") as f_out:
        reader = make_reader(f_in)
        writer = Writer(f_out)
        writer.start(profile="ros2", library="rewrite_urdf_mcap")

        # Map original channel_id/schema_id → new ones in our writer
        schema_map: dict[int, int] = {}
        channel_map: dict[int, int] = {}

        for schema in reader.get_summary().schemas.values():
            new_id = writer.register_schema(
                name=schema.name,
                encoding=schema.encoding,
                data=schema.data,
            )
            schema_map[schema.id] = new_id

        for channel in reader.get_summary().channels.values():
            new_id = writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_map[channel.schema_id],
                metadata=dict(channel.metadata),
            )
            channel_map[channel.id] = new_id

        # Stream every message through, rewriting /robot_description
        for schema, channel, message in reader.iter_messages():
            total_msgs += 1
            data = message.data
            if channel.topic == "/robot_description":
                new_data = rewrite_bytes(data)
                if new_data != data:
                    rewritten_msgs += 1
                data = new_data

            writer.add_message(
                channel_id=channel_map[channel.id],
                log_time=message.log_time,
                data=data,
                publish_time=message.publish_time,
                sequence=message.sequence,
            )

        writer.finish()

    print(f"Done. Total messages: {total_msgs}, rewritten: {rewritten_msgs}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
