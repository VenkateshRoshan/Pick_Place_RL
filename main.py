from env import PickAndPlaceEnv

if __name__ == "__main__":
    env = PickAndPlaceEnv()
    observation = env.reset()
    
    for _ in range(100):
        action = env.action_space.sample()  # Sample random action
        observation, reward, done, info = env.step(action)
        print(f"Observation: {observation}, Reward: {reward}, Done: {done}")
        
        if done:
            observation = env.reset()

    env.close()
