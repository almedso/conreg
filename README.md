# sensact

[![crate](https://img.shields.io/crates/v/num-traits.svg)](https://crates.io/crates/sensact)
[![documentation](https://docs.rs/num-traits/badge.svg)](https://docs.rs/sensact)
[![minimum rustc 1.8](https://img.shields.io/badge/rustc-1.8+-red.svg)](https://rust-lang.github.io/rfcs/2495-min-rust-version.html)
[![build status](https://github.com/rust-num/num-traits/workflows/master/badge.svg)](https://github.com/rust-num/num-traits/actions)

Sensors and Actuators in Rust.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
sensact = "0.1"
```

and this to your crate root:

```rust
use sensact::*;
```

## Features

This crate can be used without the standard library (`#![no_std]`) by disabling
the default `std` feature. Use this in `Cargo.toml`:

## Releases

Release notes are available in [RELEASES.md](RELEASES.md).

## Compatibility

The `sensact` crate is tested for rustc 1.8 and greater.

## License

Licensed under

 * [MIT license](http://opensource.org/licenses/MIT)

at your option.
