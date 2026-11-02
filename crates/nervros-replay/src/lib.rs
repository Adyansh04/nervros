//! Real provider exchanges replayed through NervROS's client and turn loop: the tests are in
//! `tests/`, the recordings in `tests/cassettes/`. A crate of its own because rig's recorder keeps
//! JSON keys in insertion order, which `nervros-core`'s tests must not inherit.
