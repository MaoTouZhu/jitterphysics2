using System.Reflection;
using System.Threading;
using Jitter2.DataStructures;
using Jitter2.Unmanaged;

namespace JitterTests.Robustness;

public class ArbiterCreationTests
{
    private static readonly TimeSpan Timeout = TimeSpan.FromSeconds(10);

    [Test]
    public void ConcurrentCreation_WithDuplicateKeysAndBufferGrowth_PublishesUniqueInitializedArbiters()
    {
        using var world = new World();
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        var buffer = GetField<PartitionedBuffer<ContactData>>(world, "memContacts");
        int pairCount = GetField<int>(buffer, "size") * 2 + 1;
        Arbiter?[] expected = new Arbiter[pairCount];
        int duplicates = 0;

        System.Threading.Tasks.Parallel.For(0, pairCount * 4, i =>
        {
            int index = i % pairCount;
            world.GetOrCreateArbiter((ulong)index, ulong.MaxValue, body1, body2, out var arbiter);
            var previous = Interlocked.CompareExchange(ref expected[index], arbiter, null);
            if (previous != null && !ReferenceEquals(previous, arbiter))
                Interlocked.Increment(ref duplicates);

            // 在其他线程分配和调整缓冲区大小的同时，执行接触写入。
            world.RegisterContact(arbiter, JVector.Zero, JVector.Zero, JVector.UnitY);
        });

        Assert.That(duplicates, Is.Zero);
        Assert.That(buffer.Count, Is.EqualTo(pairCount));
        Assert.That(GetField<SlimBag<Arbiter>>(world, "deferredArbiters"),
            Is.EquivalentTo(expected));

        for (int i = 0; i < pairCount; i++)
        {
            Assert.That(world.GetArbiter((ulong)i, ulong.MaxValue, out var arbiter), Is.True);
            Assert.That(arbiter, Is.SameAs(expected[i]));
            Assert.That(arbiter!.Body1, Is.SameAs(body1));
            Assert.That(arbiter.Body2, Is.SameAs(body2));
            Assert.That(arbiter.Handle.Data.Key, Is.EqualTo(new ArbiterKey((ulong)i, ulong.MaxValue)));
            Assert.That(arbiter.Handle.Data.Body1, Is.EqualTo(body1.Handle));
            Assert.That(arbiter.Handle.Data.Body2, Is.EqualTo(body2.Handle));
            Assert.That(arbiter.Handle.Data.UsageMask & ContactData.MaskContactAll, Is.Not.Zero);
        }
    }

    [TestCase(false)]
    [TestCase(true)]
    public void Removal_OnSteppingThread_RecyclesArbitersForWorkers(bool automaticRemoval)
    {
        using var world = new World { Gravity = JVector.Zero, AllowDeactivation = false };
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        Arbiter? first = null;

        for (int round = 0; round < 8; round++)
        {
            Arbiter? created = null;
            RunOnWorker(() => world.GetOrCreateArbiter(1, 2, body1, body2, out created));
            first ??= created;
            Assert.That(created, Is.SameAs(first));

            // 在移除仲裁器之前先处理延迟的岛注册。
            world.Step((Real)(1.0 / 60.0), multiThread: false);
            if (automaticRemoval)
                world.Step((Real)(1.0 / 60.0), multiThread: false);
            else
                world.Remove(created);

            Assert.That(world.GetArbiter(1, 2, out _), Is.False);
            Assert.That(created.Handle.IsZero, Is.True);
            Assert.That(created.Body1, Is.Null);
            Assert.That(created.Body2, Is.Null);
            Assert.That(body1.Contacts, Is.Empty);
            Assert.That(body2.Contacts, Is.Empty);
        }
    }

    [Test]
    public void RemovingArbiterBeforeFirstStep_CancelsDeferredCreation()
    {
        using var world = new World { Gravity = JVector.Zero };
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        int beginCount = 0;
        body1.BeginCollide += _ => beginCount++;

        world.GetOrCreateArbiter(1, 2, body1, body2, out var arbiter);
        world.Remove(arbiter);

        Assert.That(GetField<SlimBag<Arbiter>>(world, "deferredArbiters"), Is.Empty);
        Assert.DoesNotThrow(() => world.Step((Real)(1.0 / 60.0), multiThread: false));
        Assert.That(beginCount, Is.Zero);
    }

    [Test]
    public void RemovingBodyBeforeFirstStep_RemovesItsDeferredArbiters()
    {
        using var world = new World { Gravity = JVector.Zero };
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        var body3 = world.CreateRigidBody();
        int survivingBegins = 0;
        body3.BeginCollide += _ => survivingBegins++;
        world.GetOrCreateArbiter(1, 2, body1, body2, out _);
        world.GetOrCreateArbiter(3, 4, body1, body3, out _);
        world.GetOrCreateArbiter(5, 6, body2, body3, out var surviving);

        world.Remove(body1);

        Assert.That(world.GetArbiter(1, 2, out _), Is.False);
        Assert.That(world.GetArbiter(3, 4, out _), Is.False);
        Assert.That(GetField<SlimBag<Arbiter>>(world, "deferredArbiters"), Is.EquivalentTo(new[] { surviving }));
        Assert.DoesNotThrow(() => world.Step((Real)(1.0 / 60.0), multiThread: false));
        Assert.That(survivingBegins, Is.EqualTo(1));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void RemovingShapeBeforeFirstStep_RemovesItsDeferredArbiters(bool removeMany)
    {
        using var world = new World { Gravity = JVector.Zero };
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        var shape1 = new SphereShape(1);
        var shape2 = new SphereShape(1);
        body1.AddShape(shape1);
        body2.AddShape(shape2);
        body2.Position = new JVector(10, 0, 0);
        world.GetOrCreateArbiter(shape1.ShapeId, shape2.ShapeId, body1, body2, out _);

        if (removeMany)
            body1.RemoveShapes([shape1], MassInertiaUpdateMode.Preserve);
        else
            body1.RemoveShape(shape1, MassInertiaUpdateMode.Preserve);

        Assert.That(world.GetArbiter(shape1.ShapeId, shape2.ShapeId, out _), Is.False);
        Assert.That(GetField<SlimBag<Arbiter>>(world, "deferredArbiters"), Is.Empty);
        Assert.DoesNotThrow(() => world.Step((Real)(1.0 / 60.0), multiThread: false));
    }

    [Test]
    public void Recycling_IsScopedToTheOwningWorld()
    {
        using var world1 = new World();
        using var world2 = new World();
        var body1 = world1.CreateRigidBody();
        var body2 = world1.CreateRigidBody();
        world1.GetOrCreateArbiter(1, 2, body1, body2, out var recycled);
        world1.Step((Real)(1.0 / 60.0), multiThread: false);
        world1.Remove(recycled);

        world2.GetOrCreateArbiter(1, 2, world2.CreateRigidBody(), world2.CreateRigidBody(), out var other);
        Assert.That(other, Is.Not.SameAs(recycled));

        Arbiter? reused = null;
        RunOnWorker(() => world1.GetOrCreateArbiter(3, 4, body2, body1, out reused));
        Assert.That(reused, Is.SameAs(recycled));
        Assert.That(reused!.Body1, Is.SameAs(body2));
        Assert.That(reused.Body2, Is.SameAs(body1));
        Assert.That(reused.Handle.Data.Key, Is.EqualTo(new ArbiterKey(3, 4)));
    }

    [Test]
    public void AllocationFailure_ReturnsRentedArbiterAndAllowsRetry()
    {
        using var world = new World();
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        var buffer = GetField<PartitionedBuffer<ContactData>>(world, "memContacts");
        FieldInfo sizeField = typeof(PartitionedBuffer<ContactData>).GetField("size", BindingFlags.Instance | BindingFlags.NonPublic)!;
        PropertyInfo countProperty = typeof(PartitionedBuffer<ContactData>).GetProperty(nameof(buffer.Count))!;
        int size = (int)sizeField.GetValue(buffer)!;

        try
        {
            // 在 Allocate 的带检查扩容触及内存之前触发它，从而既不必分配
            // 数 GB 内存，也无需向热路径引入故障钩子。
            sizeField.SetValue(buffer, int.MaxValue);
            countProperty.SetValue(buffer, int.MaxValue);
            Assert.Throws<OverflowException>(() =>
                world.GetOrCreateArbiter(1, 2, body1, body2, out _));
        }
        finally
        {
            sizeField.SetValue(buffer, size);
            countProperty.SetValue(buffer, 0);
        }

        AssertRolledBackAndRetry(world, body1, body2);
    }

#if !DEBUG
    // Debug 前置条件检查会在分配之前拒绝该故障激励。
    [Test]
    public void InitializationFailure_FreesAllocatedContactAndAllowsRetry()
    {
        using var world = new World();
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();

        // 仅作为故障激励：这并不是要验证空刚体的契约。
        // Init 在分配之后、仲裁器进入延迟包之前失败。
        Assert.Throws<NullReferenceException>(() =>
            world.GetOrCreateArbiter(1, 2, body1, null!, out _));

        AssertRolledBackAndRetry(world, body1, body2);
    }
#endif

    [Test]
    public void PublicationFailure_RemovesDeferredArbiterAndAllowsRetry()
    {
        using var world = new World();
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        var buffer = GetField<PartitionedBuffer<ContactData>>(world, "memContacts");
        bool heldBufferLock = true;
        var comparer = InstallFailingComparer(world, new ArbiterKey(1, 2));
        comparer.BeforeFailure = () => heldBufferLock = Monitor.IsEntered(buffer);

        var exception = Assert.Throws<OutOfMemoryException>(() =>
            world.GetOrCreateArbiter(1, 2, body1, body2, out _));

        Assert.That(exception, Is.SameAs(comparer.Failure));
        Assert.That(heldBufferLock, Is.False, "Dictionary insertion must not hold the shared buffer lock.");
        AssertRolledBackAndRetry(world, body1, body2);
    }

    [Test]
    public void PublicationFailure_WaitsForContactReadersBeforeMovingLiveContact()
    {
        using var world = new World();
        var body1 = world.CreateRigidBody();
        var body2 = world.CreateRigidBody();
        var buffer = GetField<PartitionedBuffer<ContactData>>(world, "memContacts");
        var shards = GetField<ShardedDictionary<ArbiterKey, Arbiter>>(world, "arbiters");
        ArbiterKey failedKey = new(1, 2);
        ArbiterKey liveKey = new(3, 4);
        while (shards.GetLock(failedKey) == shards.GetLock(liveKey))
            liveKey = new ArbiterKey(liveKey.Key1 + 1, liveKey.Key2 + 1);

        using ManualResetEventSlim publishing = new(false);
        using ManualResetEventSlim failPublication = new(false);
        using ManualResetEventSlim completed = new(false);
        var comparer = InstallFailingComparer(world, failedKey);
        comparer.BeforeFailure = () =>
        {
            publishing.Set();
            if (!failPublication.Wait(Timeout)) throw new TimeoutException("Publication was not released.");
        };

        Exception? workerException = null;
        Thread worker = new(() =>
        {
            try { world.GetOrCreateArbiter(1, 2, body1, body2, out _); }
            catch (Exception exception) { workerException = exception; }
            finally { completed.Set(); }
        }) { IsBackground = true };
        worker.Start();

        Arbiter? live = null;
        try
        {
            Assert.That(publishing.Wait(Timeout), Is.True);
            // 在失败仲裁器的发布暂停期间，另一个分片必须能够创建接触。
            // 其接触将由 Free 移动。
            world.GetOrCreateArbiter(liveKey.Key1, liveKey.Key2, body1, body2, out live);
            Assert.That(buffer.GetIndex(live.Handle), Is.EqualTo(1));

            buffer.ResizeLock.EnterReadLock();
            try
            {
                ref ContactData data = ref live.Handle.Data;
                failPublication.Set();
                // 观察写者意图，使该检查不受异常与回滚 Free 之间
                // 调度延迟的影响。
                Assert.That(SpinWait.SpinUntil(() =>
                    GetField<int>(buffer.ResizeLock, "writer") != 0 || completed.IsSet, Timeout), Is.True);
                Assert.That(completed.IsSet, Is.False, "Rollback must wait for the contact reader.");
                data.Friction = (Real)7.5;
            }
            finally
            {
                buffer.ResizeLock.ExitReadLock();
            }
        }
        finally
        {
            failPublication.Set();
            Assert.That(worker.Join(Timeout), Is.True, "Rollback worker did not finish.");
        }

        Assert.That(workerException, Is.SameAs(comparer.Failure));
        Assert.That(buffer.Count, Is.EqualTo(1));
        Assert.That(buffer.GetIndex(live!.Handle), Is.Zero);
        Assert.That(live.Handle.Data.Key, Is.EqualTo(liveKey));
        Assert.That(live.Handle.Data.Friction, Is.EqualTo((Real)7.5));
        Assert.That(GetField<SlimBag<Arbiter>>(world, "deferredArbiters"), Is.EquivalentTo(new[] { live }));
        Assert.That(world.GetArbiter(1, 2, out _), Is.False);
        Assert.That(world.GetArbiter(liveKey.Key1, liveKey.Key2, out var found), Is.True);
        Assert.That(found, Is.SameAs(live));
    }

    private static void AssertRolledBackAndRetry(World world, RigidBody body1, RigidBody body2)
    {
        Assert.That(world.GetArbiter(1, 2, out _), Is.False);
        Assert.That(GetField<PartitionedBuffer<ContactData>>(world, "memContacts").Count, Is.Zero);
        Assert.That(GetField<SlimBag<Arbiter>>(world, "deferredArbiters"), Is.Empty);
        var arbiterPool = GetField<Stack<Arbiter>>(world, "arbiterPool");
        Assert.That(arbiterPool, Has.Count.EqualTo(1));
        Arbiter recycled = arbiterPool.Peek();
        Assert.That(recycled, Is.Not.Null);
        Assert.That(recycled.Handle.IsZero, Is.True);
        Assert.That(recycled.Body1, Is.Null);
        Assert.That(recycled.Body2, Is.Null);

        world.GetOrCreateArbiter(1, 2, body1, body2, out var retried);
        Assert.That(retried, Is.SameAs(recycled));
        Assert.That(arbiterPool, Is.Empty);
        Assert.That(GetField<PartitionedBuffer<ContactData>>(world, "memContacts").Count, Is.EqualTo(1));
        world.Step((Real)(1.0 / 60.0), multiThread: false);
        Assert.That(body1.Contacts, Is.EquivalentTo(new[] { retried }));
        Assert.That(body2.Contacts, Is.EquivalentTo(new[] { retried }));
    }

    private static void RunOnWorker(Action action)
    {
        Exception? workerException = null;
        Thread worker = new(() =>
        {
            try { action(); }
            catch (Exception exception) { workerException = exception; }
        }) { IsBackground = true };
        worker.Start();
        Assert.That(worker.Join(Timeout), Is.True, "Worker did not finish.");
        Assert.That(workerException, Is.Null);
    }

    private static T GetField<T>(object instance, string name)
    {
        return (T)instance.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(instance)!;
    }

    private static FailingComparer InstallFailingComparer(World world, ArbiterKey key)
    {
        var shards = GetField<ShardedDictionary<ArbiterKey, Arbiter>>(world, "arbiters");
        var dictionaries = GetField<Dictionary<ArbiterKey, Arbiter>[]>(shards, "dictionaries");
        int index = (key.GetHashCode() & 0x7FFFFFFF) % dictionaries.Length;
        Assert.That(dictionaries[index], Is.Empty);
        FailingComparer comparer = new();
        // 分配桶，使 TryGetValue 即使在字典为空时也会计算哈希。
        dictionaries[index] = new Dictionary<ArbiterKey, Arbiter>(1, comparer);
        return comparer;
    }

    private sealed class FailingComparer : IEqualityComparer<ArbiterKey>
    {
        private int hashCalls;
        public readonly OutOfMemoryException Failure = new("Injected dictionary publication failure.");
        public Action? BeforeFailure;

        public bool Equals(ArbiterKey x, ArbiterKey y) => x.Equals(y);

        public int GetHashCode(ArbiterKey key)
        {
            // 初始查找成功；随后的 Add 在插入之前失败。
            if (++hashCalls == 2)
            {
                BeforeFailure?.Invoke();
                throw Failure;
            }

            return key.GetHashCode();
        }
    }
}
