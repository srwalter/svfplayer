use std::iter::zip;

use jtag_taps::cable::Cable;
use jtag_taps::statemachine::{JtagSM, JtagState};
use svf::{Command, ParseError, RunClock, RunTestForm, State, TRSTMode};

struct Svf {
    endir: JtagState,
    enddr: JtagState,
    end_state: JtagState,
    run_state: JtagState,
    sir_smask: Vec<u8>,
    sir_mask: Vec<u8>,
    sir_tdi: Vec<u8>,
    sdr_smask: Vec<u8>,
    sdr_mask: Vec<u8>,
    sdr_tdi: Vec<u8>,
}

impl Svf {
    fn new() -> Self {
        Svf {
            endir: JtagState::Idle,
            enddr: JtagState::Idle,
            end_state: JtagState::Idle,
            run_state: JtagState::Idle,
            sir_smask: vec![],
            sir_mask: vec![],
            sir_tdi: vec![],
            sdr_smask: vec![],
            sdr_mask: vec![],
            sdr_tdi: vec![],
        }
    }

    fn to_jtag_state(state: State) -> JtagState {
        match state {
            State::RESET => JtagState::Reset,
            State::IDLE => JtagState::Idle,
            State::DRSELECT => JtagState::SelectDR,
            State::DRCAPTURE => JtagState::CaptureDR,
            State::DRSHIFT => JtagState::ShiftDR,
            State::DREXIT1 => JtagState::Exit1DR,
            State::DRPAUSE => JtagState::PauseDR,
            State::DREXIT2 => JtagState::Exit2DR,
            State::DRUPDATE => JtagState::UpdateDR,
            State::IRSELECT => JtagState::SelectIR,
            State::IRCAPTURE => JtagState::CaptureIR,
            State::IRSHIFT => JtagState::ShiftIR,
            State::IREXIT1 => JtagState::Exit1IR,
            State::IRPAUSE => JtagState::PauseIR,
            State::IREXIT2 => JtagState::Exit2IR,
            State::IRUPDATE => JtagState::UpdateIR,
        }
    }

    fn run_command<T: std::ops::DerefMut<Target=dyn Cable>>(&mut self, cmd: Command, sm: &mut JtagSM<T>) {
        match cmd {
            Command::TRST(mode) => {
                if mode != TRSTMode::Off {
                    eprintln!("TRST control not implemented");
                    unimplemented!();
                }
            }
            Command::EndDR(state) => self.enddr = Self::to_jtag_state(state),
            Command::EndIR(state) => self.endir = Self::to_jtag_state(state),
            Command::State{path, end} => {
                assert!(path.is_none());
                sm.change_mode(Self::to_jtag_state(end));
            }
            Command::HIR(pattern) => {
                if pattern.length != 0 {
                    eprintln!("TIR not implemented");
                    unimplemented!();
                }
            }
            Command::HDR(pattern) => {
                if pattern.length != 0 {
                    eprintln!("TIR not implemented");
                    unimplemented!();
                }
            }
            Command::TIR(pattern) => {
                if pattern.length != 0 {
                    eprintln!("TIR not implemented");
                    unimplemented!();
                }
            }
            Command::TDR(pattern) => {
                if pattern.length != 0 {
                    eprintln!("TIR not implemented");
                    unimplemented!();
                }
            }
            Command::SIR(pattern) => {
                if let Some(smask) = pattern.smask {
                    self.sir_smask = smask
                }
                if let Some(mask) = pattern.mask {
                    self.sir_mask = mask
                }
                if let Some(tdi) = pattern.tdi {
                    self.sir_tdi = tdi
                }
                let mut len = (pattern.length % 8) as u8;
                if len == 0 {
                    len = 8;
                }

                let mut buf = vec![];
                for (tdi, mask) in zip(&self.sir_tdi, &self.sir_smask) {
                    buf.push(tdi & mask);
                }
                sm.change_mode(JtagState::ShiftIR);
                let read = sm.cable.read_write_data(&buf, len, true);
                sm.change_mode(self.endir);

                if let Some(tdo) = pattern.tdo {
                    for (r, (tdo, mask)) in zip(&read, zip(&tdo, &self.sir_mask)) {
                        assert_eq!(*r, tdo & mask);
                    }
                }
            }
            Command::SDR(pattern) => {
                if let Some(smask) = pattern.smask {
                    self.sdr_smask = smask
                }
                if let Some(mask) = pattern.mask {
                    self.sdr_mask = mask
                }
                if let Some(tdi) = pattern.tdi {
                    self.sdr_tdi = tdi
                }
                let mut len = (pattern.length % 8) as u8;
                if len == 0 {
                    len = 8;
                }

                let mut buf = vec![];
                for (tdi, mask) in std::iter::zip(self.sdr_tdi.iter(), self.sdr_smask.iter()) {
                    buf.push(tdi & mask);
                }
                sm.change_mode(JtagState::ShiftDR);
                let read = sm.cable.read_write_data(&buf, len, true);
                sm.change_mode(self.enddr);

                if let Some(tdo) = pattern.tdo {
                    for (r, (tdo, mask)) in zip(&read, zip(&tdo, &self.sdr_mask)) {
                        assert_eq!(r & mask, tdo & mask);
                    }
                }
            }
            Command::RunTest{run_state, form, end_state} => {
                if let Some(end_state) = end_state {
                    self.end_state = Self::to_jtag_state(end_state);
                }
                if let Some(run_state) = run_state {
                    self.run_state = Self::to_jtag_state(run_state);
                }
                sm.change_mode(self.run_state);
                match form {
                    RunTestForm::Clocked { mut run_count, run_clk, time } => {
                        assert!(time.is_none());
                        assert_eq!(run_clk, RunClock::TCK);
                        while run_count > 0 {
                            if run_count > 100 {
                                sm.cable.change_mode(&vec![0; 100], true);
                                println!("runtest");
                                run_count -= 100;
                            } else {
                                sm.cable.change_mode(&vec![0; run_count as usize], true);
                                break;
                            }
                        }
                    }
                    RunTestForm::Timed(_) => unimplemented!(),
                }
                sm.change_mode(self.end_state);
            }
            Command::Frequency(_) => eprintln!("Warning: frequency control not implemented"),
            _ => {
                eprintln!("unimplemented command: {}", cmd);
                unimplemented!();
            }
        }
    }
}

fn run_svf<T: std::ops::DerefMut<Target=dyn Cable>>(sm: &mut JtagSM<T>, contents: &str) -> Result<(),ParseError> {
    let mut svf = Svf::new();

    for cmd in svf::parse_iter(&contents) {
        let cmd = cmd?;
        println!("{}", cmd);
        svf.run_command(cmd, sm);
    }
    Ok(())
}

fn main() {
    let contents = std::fs::read_to_string("example.svf").expect("read");
    let cable = jtag_taps::cable::new_from_string("easyflash3", 100 << 10).expect("cable");
    let mut jtag = JtagSM::new(cable);
    run_svf(&mut jtag, &contents).expect("svf");
}
