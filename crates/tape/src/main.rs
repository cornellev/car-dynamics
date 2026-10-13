use cornellev_dynamics_tape::{assert_tape, fixture_path, load};

fn main() {
    let path = fixture_path();
    let tape = load(&path).unwrap_or_else(|error| {
        eprintln!("{error}");
        std::process::exit(1);
    });
    if let Err(error) = assert_tape(&tape) {
        eprintln!("{error}");
        std::process::exit(1);
    }
    println!("ok {}", tape.cases.len());
}
