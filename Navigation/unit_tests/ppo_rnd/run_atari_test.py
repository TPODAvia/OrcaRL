# Copyright 2022 The Deep RL Zoo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or  implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================
"""Tests for PPO-RND."""
from pathlib import Path
import shutil
import multiprocessing
from absl import flags
from absl.testing import flagsaver
from absl.testing import absltest
from deep_rl_zoo.ppo_rnd import run_atari

FLAGS = flags.FLAGS
FLAGS.checkpoint_dir = '/tmp/e2e_test_checkpoint'
FLAGS.results_csv_path = ''
FLAGS.use_tensorboard = False
FLAGS.init_rnd_obs_steps = 20
FLAGS.max_episode_steps = 100
FLAGS.num_train_steps = 500
FLAGS.num_eval_steps = 200
FLAGS.num_iterations = 1


class RunAtariGameTest(absltest.TestCase):
    def setUp(self):
        super().setUp()
        self.checkpoint_dir = Path(FLAGS.checkpoint_dir)

    @flagsaver.flagsaver
    def test_can_run_agent(self):
        FLAGS.environment_name = 'MontezumaRevenge'
        FLAGS.num_actors = 2
        FLAGS.unroll_length = 8
        FLAGS.update_k = 2
        FLAGS.clip_grad = True

        run_atari.main(None)

    def tearDown(self) -> None:
        # Clean up
        try:
            shutil.rmtree(self.checkpoint_dir)
        except FileNotFoundError:
            pass


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')
    absltest.main()
