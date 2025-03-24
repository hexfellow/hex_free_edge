/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-15
 ****************************************************************/

#include "hex_free_edge/data_interface/data_interface.h"
#include "hex_free_edge/hex_free_edge.h"

using hex::postprocess::DataInterface;
using hex::postprocess::HexFreeEdge;
using hex::postprocess::HexLogLevel;

const char kNodeName[] = "hex_free_edge";

void TimeHandle() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static HexFreeEdge& hex_free_edge = HexFreeEdge::GetSingleton();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (hex_free_edge.Init()) {
        data_interface.Log(HexLogLevel::kInfo, "%s : Init Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Init Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (hex_free_edge.Work()) {
        // data_interface.Log(HexLogLevel::kInfo, "%s : Work Succeded",
        // kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Work Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(HexLogLevel::kError, "%s : Unknown State", kNodeName);
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

int main(int argc, char** argv) {
  DataInterface& data_interface = DataInterface::GetSingleton();
  data_interface.Init(argc, argv, kNodeName, 20.0, TimeHandle);

  data_interface.Work();

  data_interface.Deinit();
  return 0;
}
