from gym.envs.registration import register
register(
    id='BulletCartpole-v0',
    entry_point='new.envs:BulletCartpole'
)