# Station configuration sources

This directory contains commissioning-source data and examples.  Nothing in
this directory is compiled into the STCAS firmware.

The intended installation flow is:

1. An authorized commissioning tool reads the station source data.
2. The tool validates the station identity, field mapping, routes, topology,
   profiles, and other required sections.
3. The tool emits a versioned, big-endian `STCF` binary blob.
4. The blob is approved, signed, installed in nonvolatile memory, and read
   back for verification.
5. The generic `station_config` firmware module validates and activates it.

After activation, the input-card driver derives the participating card set and
the physical card/channel-to-logical-ID mapping from the blob.  Input-card
frames are rejected while no validated configuration is active, and logical
values remain unavailable until their physical card has supplied a snapshot.
Changing the active station ID or configuration revision clears all cached
input values and CAN de-duplication state.

The nonvolatile-memory loader is not implemented yet.  For bench testing,
`Drivers/station_config_test.c` constructs the incomplete Nawandgi example as
an STCF blob in RAM and passes it through the production validator.  This path
is controlled by `STATION_CONFIG_USE_NAWANDGI_TEST_CONFIG` and must be set to
zero in production/NVM-loader builds.  It does not establish that the example
data is approved commissioning data.

The current binary format uses a 32-byte fixed header followed by 16-byte
section descriptors.  All multibyte integers are big-endian.  The CRC-32 is
calculated over the complete blob while treating header bytes 20 through 23
as zero.  See `include/station_config.h` for the constants and public API.

Field-input section version 2 uses fixed 32-byte entries: logical ID (2),
card (1), channel (1), type (1), raw encoding (1), reserved flags (2), and
name (24).  Input qualification and stale-timeout values are deliberately not
part of the station configuration; those behaviours require a separately
approved system requirement and safety allocation before implementation.

Signal, point, and track section version 1 records bind station objects to
logical field-input IDs.  Signal records contain optional RECR, HECR, DECR,
and HHECR input bindings; a zero ID means that relay is not provided.  Point
records bind the A and B detection inputs for one numbered point.  Track
records bind one track-circuit or axle-counter input.  These sections define
identity and input relationships only; they do not yet implement signal-aspect
deduction, route locking, movement authority, or other interlocking rules.

The files under `examples/nawandgi` are incomplete engineering inputs for
development and testing.  They are not approved installation data and must
not be converted into an operational configuration until every item marked
`requires_confirmation` has been resolved against controlled station
documents and the physical input allocation.

For bench testing, the Nawandgi example deliberately maps logical inputs
1-32 to card 1 channels 0-31, inputs 33-64 to card 2 channels 0-31, and
inputs 65-93 to card 3 channels 0-28.  Card 3 channels 29-31 are unused.
This sequential allocation is only a test fixture.  A commissioned station
configuration must replace it with the actual relay-to-card wiring schedule.
