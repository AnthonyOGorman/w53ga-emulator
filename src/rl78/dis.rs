//! RL78-S3 disassembler (debug aid), same opcode maps as `cpu.rs`.

const R8: [&str; 8] = ["X", "A", "C", "B", "E", "D", "L", "H"];
const RP: [&str; 4] = ["AX", "BC", "DE", "HL"];
const ALU: [&str; 8] = ["ADD", "ADDC", "SUB", "SUBC", "CMP", "AND", "OR", "XOR"];

fn saddr(b: u8) -> String { format!("0x{:05X}", if b < 0x20 { 0xFFF00 | b as u32 } else { 0xFFE00 | b as u32 }) }
fn sfr(b: u8) -> String { format!("0x{:05X}", 0xFFF00 | b as u32) }

/// Disassemble one instruction at `pc` using `rd` to fetch bytes; returns (text, length).
pub fn dis(rd: &dyn Fn(u32) -> u8, pc: u32) -> (String, u32) {
    let pp = std::cell::Cell::new(pc);
    let mut next = || { let v = rd(pp.get()); pp.set(pp.get() + 1); v };
    let mut es = "";
    let mut op = next();
    if op == 0x11 { es = "ES:"; op = next(); }
    let mut w = |n: &mut dyn FnMut() -> u8| -> u16 { let l = n() as u16; l | ((n() as u16) << 8) };
    let (hi, lo) = (op >> 4, op & 15);
    let abs = |a: u16| format!("{es}!0x{a:04X}");
    let t: String = match op {
        0x00 => "NOP".into(),
        0x01 | 0x03 | 0x05 | 0x07 | 0x23 | 0x25 | 0x27 | 0x43 | 0x45 | 0x47 => format!("{} AX,{}", ["ADDW", "", "SUBW", "", "CMPW"][hi as usize], RP[(lo / 2) as usize]),
        0x02 | 0x22 | 0x42 => { let a = w(&mut next); format!("{} AX,{}", ["ADDW", "", "SUBW", "", "CMPW"][hi as usize], abs(a)) }
        0x04 | 0x24 | 0x44 => { let a = w(&mut next); format!("{} AX,#0x{a:04X}", ["ADDW", "", "SUBW", "", "CMPW"][hi as usize]) }
        0x06 | 0x26 | 0x46 => { let b = next(); format!("{} AX,{}", ["ADDW", "", "SUBW", "", "CMPW"][hi as usize], saddr(b)) }
        0x10 => format!("ADDW SP,#0x{:02X}", next()),
        0x20 => format!("SUBW SP,#0x{:02X}", next()),
        0x12 => "MOVW BC,AX".into(), 0x13 => "MOVW AX,BC".into(), 0x14 => "MOVW DE,AX".into(), 0x15 => "MOVW AX,DE".into(),
        0x16 => "MOVW HL,AX".into(), 0x17 => "MOVW AX,HL".into(),
        0x30 | 0x32 | 0x34 | 0x36 => { let a = w(&mut next); format!("MOVW {},#0x{a:04X}", RP[(lo / 2) as usize]) }
        0x33 | 0x35 | 0x37 => format!("XCHW AX,{}", RP[(lo / 2) as usize]),
        0x08 => "XCH A,X".into(),
        0x09 => { let a = w(&mut next); format!("MOV A,{es}0x{a:04X}[B]") }
        0x18 => { let a = w(&mut next); format!("MOV {es}0x{a:04X}[B],A") }
        0x19 => { let a = w(&mut next); let b = next(); format!("MOV {es}0x{a:04X}[B],#0x{b:02X}") }
        0x28 => { let a = w(&mut next); format!("MOV {es}0x{a:04X}[C],A") }
        0x29 => { let a = w(&mut next); format!("MOV A,{es}0x{a:04X}[C]") }
        0x38 => { let a = w(&mut next); let b = next(); format!("MOV {es}0x{a:04X}[C],#0x{b:02X}") }
        0x39 => { let a = w(&mut next); let b = next(); format!("MOV {es}0x{a:04X}[BC],#0x{b:02X}") }
        0x48 => { let a = w(&mut next); format!("MOV {es}0x{a:04X}[BC],A") }
        0x49 => { let a = w(&mut next); format!("MOV A,{es}0x{a:04X}[BC]") }
        0x58 | 0x68 | 0x78 => { let a = w(&mut next); format!("MOVW {es}0x{a:04X}[{}],AX", ["B", "C", "BC"][(hi - 5) as usize]) }
        0x59 | 0x69 | 0x79 => { let a = w(&mut next); format!("MOVW AX,{es}0x{a:04X}[{}]", ["B", "C", "BC"][(hi - 5) as usize]) }
        0x0A | 0x1A | 0x2A | 0x3A | 0x4A | 0x5A | 0x6A | 0x7A => { let b = next(); let v = next(); format!("{} {},#0x{v:02X}", ALU[hi as usize], saddr(b)) }
        _ if hi <= 7 && lo >= 0xB => {
            let src = match lo { 0xB => saddr(next()), 0xC => format!("#0x{:02X}", next()), 0xD => format!("{es}[HL]"), 0xE => format!("{es}[HL+0x{:02X}]", next()), _ => { let a = w(&mut next); abs(a) } };
            format!("{} A,{src}", ALU[hi as usize])
        }
        0x40 => { let a = w(&mut next); let b = next(); format!("CMP {},#0x{b:02X}", abs(a)) }
        0x41 => format!("MOV ES,#0x{:02X}", next()),
        0x50..=0x57 => format!("MOV {},#0x{:02X}", R8[lo as usize], next()),
        0x60 | 0x62..=0x67 => format!("MOV A,{}", R8[lo as usize]),
        0x70 | 0x72..=0x77 => format!("MOV {},A", R8[lo as usize]),
        0x80..=0x87 => format!("INC {}", R8[lo as usize]),
        0x90..=0x97 => format!("DEC {}", R8[lo as usize]),
        0x88..=0x8F | 0x98..=0x9F => {
            let m = match lo & 7 { 0 => format!("[SP+0x{:02X}]", next()), 1 => format!("{es}[DE]"), 2 => format!("{es}[DE+0x{:02X}]", next()), 3 => format!("{es}[HL]"),
                4 => format!("{es}[HL+0x{:02X}]", next()), 5 => saddr(next()), 6 => sfr(next()), _ => { let a = w(&mut next); abs(a) } };
            if hi == 8 { format!("MOV A,{m}") } else { format!("MOV {m},A") }
        }
        0xA0 => { let a = w(&mut next); format!("INC {}", abs(a)) }
        0xB0 => { let a = w(&mut next); format!("DEC {}", abs(a)) }
        0xA2 => { let a = w(&mut next); format!("INCW {}", abs(a)) }
        0xB2 => { let a = w(&mut next); format!("DECW {}", abs(a)) }
        0xA4 => format!("INC {}", saddr(next())), 0xB4 => format!("DEC {}", saddr(next())),
        0xA6 => format!("INCW {}", saddr(next())), 0xB6 => format!("DECW {}", saddr(next())),
        0xA1 | 0xA3 | 0xA5 | 0xA7 => format!("INCW {}", RP[(lo / 2) as usize]),
        0xB1 | 0xB3 | 0xB5 | 0xB7 => format!("DECW {}", RP[(lo / 2) as usize]),
        0xA8..=0xAF | 0xB8..=0xBF => {
            let m = match lo { 8 => format!("[SP+0x{:02X}]", next()), 9 => format!("{es}[DE]"), 0xA => format!("{es}[DE+0x{:02X}]", next()), 0xB => format!("{es}[HL]"),
                0xC => format!("{es}[HL+0x{:02X}]", next()), 0xD => saddr(next()), 0xE => sfr(next()), _ => { let a = w(&mut next); abs(a) } };
            if hi == 0xA { format!("MOVW AX,{m}") } else { format!("MOVW {m},AX") }
        }
        0xC0 | 0xC2 | 0xC4 | 0xC6 => format!("POP {}", RP[(lo / 2) as usize]),
        0xC1 | 0xC3 | 0xC5 | 0xC7 => format!("PUSH {}", RP[(lo / 2) as usize]),
        0xC8 => { let d = next(); let v = next(); format!("MOV [SP+0x{d:02X}],#0x{v:02X}") }
        0xC9 => { let b = next(); let a = w(&mut next); format!("MOVW {},#0x{a:04X}", saddr(b)) }
        0xCA => { let d = next(); let v = next(); format!("MOV {es}[DE+0x{d:02X}],#0x{v:02X}") }
        0xCB => { let b = next(); let a = w(&mut next); format!("MOVW {},#0x{a:04X}", sfr(b)) }
        0xCC => { let d = next(); let v = next(); format!("MOV {es}[HL+0x{d:02X}],#0x{v:02X}") }
        0xCD => { let b = next(); let v = next(); format!("MOV {},#0x{v:02X}", saddr(b)) }
        0xCE => { let b = next(); let v = next(); if b == 0xFB { ["?", "MULHU", "MULH", "DIVHU", "?", "MACHU", "MACH", "?", "?", "?", "?", "DIVWU"].get(v as usize).unwrap_or(&"?").to_string() } else { format!("MOV {},#0x{v:02X}", sfr(b)) } }
        0xCF => { let a = w(&mut next); let v = next(); format!("MOV {},#0x{v:02X}", abs(a)) }
        0xD0..=0xD3 => format!("CMP0 {}", R8[lo as usize]),
        0xD4 => format!("CMP0 {}", saddr(next())),
        0xD5 => { let a = w(&mut next); format!("CMP0 {}", abs(a)) }
        0xD6 => "MULU X".into(),
        0xD7 => "RET".into(),
        0xD8 | 0xE8 | 0xF8 => format!("MOV {},{}", ["X", "B", "C"][(hi - 0xD) as usize], saddr(next())),
        0xD9 | 0xE9 | 0xF9 => { let a = w(&mut next); format!("MOV {},{}", ["X", "B", "C"][(hi - 0xD) as usize], abs(a)) }
        0xDA | 0xEA | 0xFA => format!("MOVW {},{}", RP[(hi - 0xC) as usize], saddr(next())),
        0xDB | 0xEB | 0xFB => { let a = w(&mut next); format!("MOVW {},{}", RP[(hi - 0xC) as usize], abs(a)) }
        0xDC..=0xDF => { let d = next() as i8; format!("{} 0x{:05X}", ["BC", "BZ", "BNC", "BNZ"][(lo - 0xC) as usize], (pp.get() as i32 + d as i32) as u32) }
        0xE0..=0xE3 => format!("ONEB {}", R8[lo as usize]),
        0xE4 => format!("ONEB {}", saddr(next())),
        0xE5 => { let a = w(&mut next); format!("ONEB {}", abs(a)) }
        0xE6 => "ONEW AX".into(), 0xE7 => "ONEW BC".into(),
        0xF0..=0xF3 => format!("CLRB {}", R8[lo as usize]),
        0xF4 => format!("CLRB {}", saddr(next())),
        0xF5 => { let a = w(&mut next); format!("CLRB {}", abs(a)) }
        0xF6 => "CLRW AX".into(), 0xF7 => "CLRW BC".into(),
        0xEC | 0xFC => { let a = w(&mut next) as u32; let h = next() as u32; format!("{} !!0x{:05X}", if op == 0xEC { "BR" } else { "CALL" }, (h << 16) | a) }
        0xED | 0xFD => { let a = w(&mut next); format!("{} !0x{a:04X}", if op == 0xED { "BR" } else { "CALL" }) }
        0xEE | 0xFE => { let d = w(&mut next) as i16; format!("{} $!0x{:05X}", if op == 0xEE { "BR" } else { "CALL" }, (pp.get() as i32 + d as i32) as u32) }
        0xEF => { let d = next() as i8; format!("BR $0x{:05X}", (pp.get() as i32 + d as i32) as u32) }
        0x61 => { let o = next(); dis2(o, &mut next, es, &pp) }
        0x71 => { let o = next(); dis3(o, &mut next, es) }
        0x31 => { let o = next(); dis4(o, &mut next, es, &pp) }
        _ => format!(".db 0x{op:02X}"),
    };
    let _ = &mut w;
    let len = pp.get() - pc;
    (t, len)
}

fn dis2(o: u8, next: &mut dyn FnMut() -> u8, es: &str, pp: &std::cell::Cell<u32>) -> String {
    let (hi, lo) = (o >> 4, o & 15);
    match o {
        0x00..=0x7F if lo <= 7 => format!("{} {},A", ALU[hi as usize], R8[lo as usize]),
        0x00..=0x7F if lo >= 8 && lo != 9 => format!("{} A,{}", ALU[hi as usize], R8[if lo == 8 { 0 } else { (lo - 8) as usize }]),
        0x09 | 0x29 | 0x49 => format!("{} AX,{es}[HL+0x{:02X}]", ["ADDW", "", "SUBW", "", "CMPW"][hi as usize], next()),
        0x59 | 0x69 | 0x79 | 0x89 => format!("{} {es}[HL+0x{:02X}]", ["", "", "", "", "", "INC", "DEC", "INCW", "DECW"][hi as usize], next()),
        0x80 | 0x90 | 0xA0 | 0xB0 | 0xC0 | 0xD0 | 0xE0 | 0xF0 => format!("{} A,{es}[HL+B]", ALU[(hi - 8) as usize]),
        0x82 | 0x92 | 0xA2 | 0xB2 | 0xC2 | 0xD2 | 0xE2 | 0xF2 => format!("{} A,{es}[HL+C]", ALU[(hi - 8) as usize]),
        _ if hi >= 8 && (4..=7).contains(&lo) => format!("CALLT [0x{:04X}]", 0x80 + (lo as u32 - 4) * 0x10 + (hi as u32 - 8) * 2),
        0x8A..=0x8F => format!("XCH A,{}", R8[(lo - 8) as usize]),
        0xA8 => format!("XCH A,{}", saddr(next())),
        0xA9 => format!("XCH A,{es}[HL+C]"),
        0xAA => { let l = next() as u16; let h = next() as u16; format!("XCH A,{es}!0x{:04X}", l | (h << 8)) }
        0xAB => format!("XCH A,{}", sfr(next())),
        0xAC => format!("XCH A,{es}[HL]"),
        0xAD => format!("XCH A,{es}[HL+0x{:02X}]", next()),
        0xAE => format!("XCH A,{es}[DE]"),
        0xAF => format!("XCH A,{es}[DE+0x{:02X}]", next()),
        0xB8 => format!("MOV ES,{}", saddr(next())),
        0xB9 => format!("XCH A,{es}[HL+B]"),
        0xC3 | 0xD3 => { let d = next() as i8; format!("{} 0x{:05X}", if o == 0xC3 { "BH" } else { "BNH" }, (pp.get() as i32 + d as i32) as u32) },
        0xC8 => "SKC".into(), 0xD8 => "SKNC".into(), 0xE8 => "SKZ".into(), 0xF8 => "SKNZ".into(), 0xE3 => "SKH".into(), 0xF3 => "SKNH".into(),
        0xC9 => format!("MOV A,{es}[HL+B]"), 0xD9 => format!("MOV {es}[HL+B],A"), 0xE9 => format!("MOV A,{es}[HL+C]"), 0xF9 => format!("MOV {es}[HL+C],A"),
        0xCA | 0xDA | 0xEA | 0xFA => format!("CALL {}", RP[(hi - 0xC) as usize]),
        0xCB => "BR AX".into(), 0xCC => "BRK".into(), 0xCD => "POP PSW".into(), 0xDD => "PUSH PSW".into(),
        0xCE => format!("MOVS {es}[HL+0x{:02X}],X", next()), 0xDE => format!("CMPS X,{es}[HL+0x{:02X}]", next()),
        0xCF | 0xDF | 0xEF | 0xFF => format!("SEL RB{}", hi - 0xC),
        0xDB => "ROR A,1".into(), 0xDC => "ROLC A,1".into(), 0xEB => "ROL A,1".into(), 0xFB => "RORC A,1".into(),
        0xEE => "ROLWC AX,1".into(), 0xFE => "ROLWC BC,1".into(),
        0xEC => "RETB".into(), 0xFC => "RETI".into(), 0xED => "HALT".into(), 0xFD => "STOP".into(),
        _ => format!(".db 0x61,0x{o:02X}"),
    }
}

fn dis3(o: u8, next: &mut dyn FnMut() -> u8, es: &str) -> String {
    let bit = (o >> 4) & 7;
    let c = o & 15;
    if o & 0x80 != 0 && c == 0 { return if bit == 0 { "SET1 CY".into() } else if bit == 4 { "NOT1 CY".into() } else { format!(".db 0x71,0x{o:02X}") }; }
    if o & 0x80 != 0 && c == 8 { return if bit == 0 { "CLR1 CY".into() } else { format!(".db 0x71,0x{o:02X}") }; }
    let opnd = if o & 0x80 == 0 {
        match c { 0 | 8 => { let l = next() as u16; let h = next() as u16; format!("{es}!0x{:04X}", l | (h << 8)) } 1..=7 => saddr(next()), _ => sfr(next()) }
    } else if c <= 7 { format!("{es}[HL]") } else { "A".into() };
    let m = match c { 0 | 2 | 0xA => "SET1 {}", 8 | 3 | 0xB => "CLR1 {}", 1 | 9 => "MOV1 {},CY", 4 | 0xC => "MOV1 CY,{}", 5 | 0xD => "AND1 CY,{}", 6 | 0xE => "OR1 CY,{}", _ => "XOR1 CY,{}" };
    m.replace("{}", &format!("{opnd}.{bit}"))
}

fn dis4(o: u8, next: &mut dyn FnMut() -> u8, es: &str, pp: &std::cell::Cell<u32>) -> String {
    let (hi, c) = (o >> 4, o & 15);
    match c {
        0..=5 => {
            let bit = hi & 7;
            let opnd = match (hi < 8, c) { (true, 0 | 2 | 4) => saddr(next()), (true, _) => "A".into(), (false, 0 | 2 | 4) => sfr(next()), (false, _) => format!("{es}[HL]") };
            let d = next() as i8;
            format!("{} {opnd}.{bit},0x{:05X}", ["BTCLR", "BTCLR", "BT", "BT", "BF", "BF"][c as usize], (pp.get() as i32 + d as i32) as u32)
        }
        7..=0xB if (1..=7).contains(&hi) => format!("{} {},{hi}", ["SHL", "SHL", "SHL", "SHR", "SAR"][(c - 7) as usize], ["C", "B", "A", "A", "A"][(c - 7) as usize]),
        0xC..=0xF if hi >= 1 => format!("{} {},{hi}", ["SHLW", "SHLW", "SHRW", "SARW"][(c - 0xC) as usize], if c == 0xC { "BC" } else { "AX" }),
        _ => format!(".db 0x31,0x{o:02X}"),
    }
}
